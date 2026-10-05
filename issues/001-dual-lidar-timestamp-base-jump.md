# 001. 🔴 双 MID360 时间戳基准跳变 → FAST_LIO 出不了里程计

- **状态**：🔴 待修
- **发现日期**：2026-10-04
- **影响面**：致命 —— 建图 / 定位 / SLAM / 导航全链路起不来
- **相关文件**：
  - `src/pb2025_sentry_nav/livox_ros_driver2/src/comm/pub_handler.cpp`（`GetEthPacketTimestamp`、`is_timestamp_sync_`、`CheckTimer`）
  - `src/pb2025_sentry_nav/livox_ros_driver2/src/lddc.cpp`（`InitCustomMsg` 写 `header.stamp`）
  - `src/pb2025_sentry_nav/livox_ros_driver2/src/lds.cpp`（`PushLidarData` / `base_time` 传递）
  - `src/pb2025_sentry_nav/FAST_LIO/src/laserMapping.cpp`（回绕判定、`first_lidar_time`、`flg_EKF_inited`、`fuse_dual_lidar_frame_once`）
  - `src/pb2025_sentry_nav/FAST_LIO/src/IMU_Processing.hpp`（`meas.imu.empty()` 提前 return）
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`（`fastlio.common.*`）

## 现象

运行 `rm_navigation_reality_launch.py`（SLAM 模式）后：

```
[fastlio_mapping-13] right lidar loop back, clear right/fused buffers. current=51.3502 last=1.79112e+09
[fastlio_mapping-13] left  lidar loop back, clear left/fused buffers.  current=51.3501 last=1.79112e+09
[fastlio_mapping-13] No point, skip this scan!
[fastlio_mapping-13] pcl::VoxelGrid::applyFilter Leaf size is too small for the input dataset. Integer indices would overflow.
[fastlio_mapping-13] Initialize the map kdtree
[fastlio_mapping-13] No Effective Points!        # 连续刷屏 184 次（2 秒内）
```

连带的下游报错（都是"果"）：

```
[global_costmap.global_costmap]: Timed out waiting for transform from gimbal_yaw_fake to map to become available,
                                tf error: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
[local_costmap.local_costmap]: Can't update static costmap layer, no map received
[mapping_mission_manager]: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[mapping_mission_manager]: 正在查找tf: gimbal_yaw_fake -> odom
[mapping_mission_manager]: bt_navigator is NOT active (state id: 1)
[pb2025_sentry_behavior_server]: SendNav2Goal: Action server with name 'navigate_to_pose' is not reachable.
[nav2_costmap_2d]: Robot is out of bounds of the costmap!
```

## 证据

- 同一路雷达的时间戳在运行中从 **`1.79112e9 s`（epoch 绝对时间）** 瞬间跌到 **`51.35 s`（雷达内部"开机后"时间）**。
- 这两个量级分别对应驱动里两条不同的取时间分支（见下）。
- 相关日志：`/home/fwy/.ros/log/2026-10-04-21-32-56-.../`。

## 根因

### (a) 驱动侧：`GetEthPacketTimestamp` 会在两种时钟源之间切换

`livox_ros_driver2/src/comm/pub_handler.cpp:265`

```cpp
uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) {
  if (timestamp_type == kTimestampTypeGptpOrPtp ||
      timestamp_type == kTimestampTypeGps) {
    return time.stamp;                                     // ← 雷达内部时间
  }
  return std::chrono::high_resolution_clock::now().time_since_epoch().count();  // ← 主机 epoch 时间
}
```

- `time_type == kTimestampTypeNoSync(0)` → 用主机 `high_resolution_clock::now()`（≈1.79e9 s）。
- `time_type == kTimestampTypeGptpOrPtp(1)` → 直接用雷达包里的 8 字节原始时间。MID360 在"PTP 模式但未真正锁主钟"时，这个值是**雷达自己的开机后 ns**（≈51.35 s），并不是 epoch。
- 同一台雷达的 `time_type` 在运行中发生切换（启动初期 NoSync → 之后报 gPTP/PTP），时间戳基准就发生了台阶式跳变。

该值经 `frame_.base_time` → `StoragePointData`/`PushLidarData` → `lddc.cpp` 的 `InitCustomMsg` 写入 `livox_msg.header.stamp`（`timestamp = pkg.base_time`，约 `lddc.cpp:366`），最终成为 FAST_LIO 看到的 `msg->header.stamp`。

### (b) 驱动侧：`is_timestamp_sync_` 是全局单变量，两雷达/IMU 互相污染

`livox_ros_driver2/src/comm/pub_handler.cpp:105`，对**任意 handle、任意包类型（点云/IMU）**统一写同一个标志；`CheckTimer()`（`:161` 起）据此在"设备时间节拍"和"主机 now() 节拍"之间切换送帧。详见 [004](./004-livox-timestamp-sync-flag.md)。

### (c) FAST_LIO 侧：无法从基准跳变中恢复

`FAST_LIO/src/laserMapping.cpp`

- 变量：`:108-109` `last_timestamp_lidar` / `..._right` / `..._left`；`:119` `is_first_lidar`。
- 回绕判定与清缓冲：右 `:838-847`、左 `:887-895`、IMU `:1014-1020`。
- 只升不降：`:853` / `:902` `last_timestamp_lidar = std::max(last_timestamp_lidar, cur_time);`
- `first_lidar_time` 只在**首帧**赋值，之后不再重设：`:1443-1444`。
- 依赖 `first_lidar_time` 的初始化判定：`:1468`

```cpp
flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;   // INIT_TIME = 0.1
```

基准跳变后 `Measures.lidar_beg_time`（≈51）远小于 `first_lidar_time`（≈1.79e9），于是 `flg_EKF_inited` **永远为 false**，`map_incremental()`（`:1171`）会每帧无脑塞点，地图被写花，配准再找不到有效平面 → `No Effective Points!` 持续刷屏 → 不产生有效 `/aft_mapped_to_init`（里程计）。

> 补充：`IMU_Processing.hpp` 的 `Process()` 在 `meas.imu.empty()` 时直接 `return`，不会刷新 `feats_undistort`，会保留上一帧点云，可能让异常状态"粘住"。

### (d) 结果链

```
时间戳基准跳变
  → FAST_LIO 回绕判定触发、清空 buffer
  → 去畸变/同步/配准失效（VoxelGrid 索引溢出 = 点云出现 inf/NaN 的征兆）
  → 持续 No Effective Points，全程无有效里程计
  → 无 odom 坐标系 TF（mapping_mission_manager / rviz 报 odom 不存在）
  → slam_toolbox 收不到 scan，不发布 map
  → global/local costmap 无 map，bt_navigator 无法激活
  → 行为树 navigate_to_pose 不可达、costmap 越界告警
```

## 方案

### 方案 A（推荐，根治）：驱动统一到"主机单一时钟"

改 `pub_handler.cpp` 的 `GetEthPacketTimestamp`，保证交给 ROS 的 `header.stamp` 永远同一基准、单调递增。

- **A1（最省事，建议先上）**：**完全忽略雷达内部时间，一律返回主机 `now()`**。
  - 即删除 `if (kTimestampTypeGptpOrPtp || kTimestampTypeGps) return time.stamp;` 分支。
  - 理由：151/159 在同一主机上，主机时钟天然统一；IMU 与点云同源（都用 151）；FAST_LIO 只用 `header.stamp` 作为 scan 起始时间，点内相对时间仍来自雷达自己的 `offset_time`，建图精度基本无损。
  - 代价：放弃雷达硬件时间戳的高精度（对单机建图无影响）。
- **A2（保留设备时间，做基准对齐）**：按 handle 维护 `offset = host_now - device_stamp`，之后一律 `device_stamp + offset` 折算成主机时间，并检测设备时间突变后重算 offset。
  - 需把 `GetEthPacketTimestamp` 从 `static` 改为能拿到 `handle` 的成员函数（调用点 `pub_handler.cpp:117`、`:143` 已有 `handle`）。
  - 复杂度高，竞赛场景收益不大。

> 一并处理 [004](./004-livox-timestamp-sync-flag.md)（`is_timestamp_sync_` 按 handle 拆分）。

### 方案 B（兜底）：FAST_LIO 的时间戳防抖 / 基准重置

1. **区分"真回绕"与"基准切换"**：`last - cur` 在几秒内 → 视为真回绕，按原逻辑清 buffer；`last - cur` 达到 `>1e3` 秒量级 → 视为基准切换，**不清 buffer**，只做重置：
   - `last_timestamp_lidar_right/left = cur_time`
   - `last_timestamp_lidar = cur_time`（不要 `max`，避免只升不降）
   - `first_lidar_time = 0`（或标记需要重设）
   - `lidar_pushed = false`
   - 位置：`laserMapping.cpp:838-853`、`:887-902`、`:1014-1020`、`:1443-1444`、`:1468`。
2. IMU 回绕分支同步处理，保证 LiDAR/IMU 基准一起切。

### 方案 C（现场应急，非代码）：设备侧避免"假 PTP"

- 若两台 MID360 本就没有接 PTP 主钟，在 Livox Viewer / 上位机里**关闭雷达时间同步**，让 `time_type` 恒为 `NoSync`，驱动就恒走 `now()`。
- 或反之，真在车上跑 `ptp4l/phc2sys` 做 PTP，让两雷达真锁主机时钟。
- 缺点：依赖人工配置、不可脚本化；竞赛现场不一定允许。

### 方案 D（防呆，配合 A/B）：保护首次建图

- 首帧若点云退化，ikdtree 会拿一张"垃圾地图"当底，之后必然一直 `No Effective Points!`。
- 建议把 `laserMapping.cpp:1482` 附近的 `feats_down_size > 5` 提高到更合理阈值（如 30），并在建图前对 `feats_undistort` 做 NaN/有限性检查。

## 落地顺序

1. 先做 **A1** + **拆 `is_timestamp_sync_`**（[004](./004-livox-timestamp-sync-flag.md)）—— 根治，改动小而集中。
2. 再做 **B** 作为兜底。
3. 顺手加 **D**。
4. **C** 留作现场应急验证。

## 验证方法

```bash
# 1) 时间戳是否单调、是否还有 1.79e9 → 51 的台阶
ros2 topic echo /livox/lidar_192_168_1_151 --field header.stamp
ros2 topic echo /livox/imu_192_168_1_151  --field header.stamp

# 2) 里程计是否稳定输出（应为 ~10Hz，不再是 0）
ros2 topic hz /aft_mapped_to_init
ros2 topic echo /Odometry --field header.stamp

# 3) TF 链是否建立（odom / map）
ros2 run tf2_ros tf2_echo odom gimbal_yaw_fake
ros2 run tf2_ros tf2_echo map  gimbal_yaw_fake
```

启动日志中应不再出现 `loop back`，`No Effective Points!` 不再持续刷屏。

## 相关文件

- `src/pb2025_sentry_nav/livox_ros_driver2/src/comm/pub_handler.cpp`（`GetEthPacketTimestamp`、`is_timestamp_sync_`、`CheckTimer`）
- `src/pb2025_sentry_nav/livox_ros_driver2/src/lddc.cpp`（`InitCustomMsg` 写 `header.stamp`）
- `src/pb2025_sentry_nav/livox_ros_driver2/src/lds.cpp`（`PushLidarData` / `base_time` 传递）
- `src/pb2025_sentry_nav/FAST_LIO/src/laserMapping.cpp`
- `src/pb2025_sentry_nav/FAST_LIO/src/IMU_Processing.hpp`
- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`
- `src/pb2025_sentry_nav/livox_ros_driver2/config/MID360_config.json`
- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json`

## 处理记录

| 日期 | 处理人 | 改动 / 结论 |
|------|--------|-------------|
| 2026-10-04 | — | 录入；尚未修改代码 |
