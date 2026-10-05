# 004. 🟡 驱动 `is_timestamp_sync_` 全局共享（两雷达互相污染）

- **状态**：🟡 观察（与 [001](./001-dual-lidar-timestamp-base-jump.md) 同源，建议一并修）
- **发现日期**：2026-10-04
- **影响面**：双雷达送帧节拍在两套时钟间横跳，叠加恶化时间戳同步
- **相关文件**：
  - `src/pb2025_sentry_nav/livox_ros_driver2/src/comm/pub_handler.cpp:105`（写标志）
  - `src/pb2025_sentry_nav/livox_ros_driver2/src/comm/pub_handler.cpp:161`（`CheckTimer` 读取并据此切换送帧节拍）

## 现象

`CheckTimer()` 的送帧节拍会在"设备时间节拍"和"主机 `now()` 节拍"之间来回切换。

## 证据

`pub_handler.cpp:105`（`OnLivoxLidarPointCloudCallback`）：

```cpp
if (data->time_type != kTimestampTypeNoSync) {
  is_timestamp_sync_.store(true);
} else {
  is_timestamp_sync_.store(false);
}
```

该回调对 **151 / 159 两台雷达的点云包和 IMU 包** 共用同一个 `is_timestamp_sync_`，谁最后到就以谁为准。

## 根因 / 疑点

- 单标志变量在"双雷达 + IMU"多来源场景下语义不成立：任一来源的 `time_type` 都会覆盖另一来源的状态。
- `CheckTimer()`（`pub_handler.cpp:161` 起）依据该标志在两种节拍间二选一，导致送帧节奏抖动。

## 方案

把 `is_timestamp_sync_` 改为按 handle 维护（`std::unordered_map<uint32_t, bool>` 或 per-handler 状态），`CheckTimer` 只看对应 handle 的状态；点云包与 IMU 包如需区分，可再细化。

建议与 [001](./001-dual-lidar-timestamp-base-jump.md) 的方案 A（驱动统一到主机时钟）一起实施。

## 验证方法

```bash
# 双雷达同时运行时观察两个 topic 的 header.stamp 是否同步、单调
ros2 topic echo /livox/lidar_192_168_1_151 --field header.stamp
ros2 topic echo /livox/lidar_192_168_1_159 --field header.stamp
```

## 处理记录

| 日期 | 处理人 | 改动 / 结论 |
|------|--------|-------------|
| 2026-10-04 | — | 录入；未修改 |
