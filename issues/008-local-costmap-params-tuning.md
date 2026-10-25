# 008. 📘 `local_costmap` 参数详解与调参指南

- **类型**：📘 参考文档（非缺陷记录，内容来自对配置与插件源码的核对）
- **更新日期**：2026-10-21
- **适用文件**：`src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:505-556`
- **适用版本**：Nav2 **Humble**（`nav2_costmap_2d` 1.1.18）
- **依赖插件**：`pb_nav2_costmap_2d::IntensityVoxelLayer`
  - `src/pb2025_sentry_nav/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp`
  - `src/pb2025_sentry_nav/pb_nav2_plugins/include/pb_nav2_plugins/layers/intensity_voxel_layer.hpp`
- **上游数据**：`/terrain_map`（`src/terrain_analysis/src/terrainAnalysis.cpp`）
- **影响面**：调参效率 / 避障安全（读懂本文可避免 60% 的“改了没反应”）

> 说明：本文把「配置里写了什么」和「代码里真正读了什么」分开讲。
> 凡是标注 **（代码验证）** 的结论，都能在仓库源码或 `/opt/ros/humble` 的头文件/库中直接对上。
> 当前配置里存在几处 **注释与实现不符** 的地方，已在 §5 单独列出。

---

## 0. 一分钟速览（当前配置的“有效工作区间”）

| 项目 | 当前值 | 实际含义 |
|------|--------|----------|
| 窗口 | `8 m × 8 m`，`rolling_window: true` | 以机器人为中心、会随odom移动的局部地图 |
| 分辨率 | `0.05 m` | 160 × 160 = 25 600 个栅格 |
| 体素 | `z_voxels: 16 × z_resolution: 0.05` | 竖直方向只覆盖 **z ∈ [0.0, 0.8) m**（见 §4.1，**关键陷阱**） |
| 障碍来源 | 仅 `/terrain_map` | `intensity` = 相对地面高度，取值天然 `< vehicleHeight = 0.5` |
| 高度过滤 | `min/max_obstacle_height = 0.03 / 2.0`（odom 系绝对 z） | 真正生效的是 **0.03 ~ 0.8**（上限被体素窗口截断） |
| 距离过滤 | `obstacle_min_range / max_range = 0.3 / 3.0` | **到 `base_footprint` 的 3D 距离**，不是水平距离；0.3 m 内盲区 |
| 组合方式 | `combination_method: 1` | **Maximum（取最大）**，不是注释里写的 Overwrite（§5.1） |
| 膨胀 | `inflation_radius 0.34` / `cost_scaling_factor 10` | 内切半径已有 0.30 m，实际只多出 **4 cm** 的梯度带（§4.3） |
| 更新 / 发布 | `25 Hz` / `8 Hz` | 25 Hz 计算、8 Hz 对外发布（可视化/RViz/`behavior_server` 的订阅） |

---

## 1. 局部代价地图在整条链路中的位置

```
MID360 ×2 → FAST-LIO(/cloud_registered, odometry)
                 │
                 ▼
        terrain_analysis  ── /terrain_map (frame: odom) ──┐
                 │  intensity = 点相对“局部地面”的高度      │
                 │  z 值 = odom 系绝对高度                  │
                 ▼                                         ▼
   local_costmap（本文件 505~556 行）  ←────────── IntensityVoxelLayer
   ├── static_layer        （/map，来自 map_server 或 slam_toolbox）
   ├── intensity_voxel_layer（动态障碍，标记 LETHAL_OBSTACLE）
   └── inflation_layer      （膨胀梯度）
                 │
                 ├─→ controller_server（MPPI 的 CostCritic / ObstaclesCritic 直接读同一份 costmap）
                 └─→ behavior_server（订阅 local_costmap/costmap_raw + published_footprint）
```

两个容易忽略的实现细节：

1. **Humble 里 `local_costmap` 是 `controller_server` 内部创建的节点**（`navigation_launch.py` 并没有单独起 costmap 节点）。
   因此 `update_frequency: 25.0` 的更新循环和 MPPI（`batch_size: 1200`、`time_steps: 50`）**跑在同一个进程里抢 CPU**。
2. 代价地图的 **更新频率（25 Hz）与发布频率（8 Hz）互不影响**：
   控制器用的是内存里的 costmap 对象，不受 8 Hz 限制；但 `behavior_server`、RViz、`costmap_raw` 的订阅者只能按 8 Hz 看到新数据 —— **恢复行为的反应延迟下限 ≈ 125 ms**。

---

## 2. 基础参数（`local_costmap.local_costmap.ros__parameters`）

### 2.1 `use_sim_time: False`（508 行）

| 项 | 值 |
|---|---|
| 类型 / 默认 | bool / `false` |
| 作用 | 是否使用 `/clock`。实车必须 `false`；仿真必须 `true` |

实车 + `false` 正确。若改成 `true` 却没仿真时钟，整个 costmap 会卡在第一帧不再更新（表现为“话题有但时间戳不动”）。

### 2.2 `update_frequency: 25.0`（509 行）

| 项 | 值 |
|---|---|
| 类型 / 单位 | double / Hz |
| 作用 | costmap 主循环频率：每周期对每个 layer 调用 `updateBounds()` + `updateCosts()` |
| 默认 | 5.0（Nav2 默认，本文件已调高） |

**调参建议**

- 25 Hz 对应 2.5 m/s 时每周期前进 0.1 m（= 2 个栅格），对 `resolution: 0.05` 刚好够用。
- 提高它 **不会** 让感知更快，只会让“同一帧点云被重复处理更多次”；真正的瓶颈是 `/terrain_map` 的发布频率（≈ LiDAR 帧率）。
- CPU 吃紧时可降到 15~20 Hz，对避障几乎无感；降到 10 Hz 以下才会明显影响高速绕障。
- 不要低于 `controller_frequency`（`controller_server` 的 20 Hz）的一半，否则控制周期会用到旧地图。

### 2.3 `publish_frequency: 8.0`（510 行）

| 项 | 值 |
|---|---|
| 类型 / 单位 | double / Hz |
| 作用 | 对外发布 `~/costmap`、`~/costmap_raw`、`~/costmap_updates`、`~/published_footprint` 的频率 |
| 默认 | 1.0 |

**影响面**：RViz 顺滑度、`behavior_server` 的碰撞检查新鲜度。
**建议**：8~10 Hz 是合理的；只在需要录 rosbag 做后处理时提到 10~15 Hz，代价是网络/序列化开销。

### 2.4 `global_frame: odom`（511 行）

| 项 | 值 |
|---|---|
| 作用 | 局部代价地图的参考系；点云与机器人都要变换到该系 |
| 默认 | `map` |

**为什么必须是 `odom`**：`rolling_window: true` 要求参考系连续、不跳变。`map` 在重定位/回环时会跳，会让窗口“瞬移”，产生幽灵障碍。
链路核对（**（代码验证）**）：`loam_interface` 把 FAST-LIO 的 `lio_odom` 重映射成 `odom`，`fake_vel_transform` 再发布 `gimbal_yaw → gimbal_yaw_fake` 的补偿变换。

### 2.5 `robot_base_frame: gimbal_yaw_fake`（512 行）

| 项 | 值 |
|---|---|
| 作用 | 机器人基座系，代价地图用它定位 footprint 中心、也用它的位姿做滚动窗口的居中 |
| 来源 | `fake_vel_transform`：`gimbal_yaw → gimbal_yaw_fake`，旋转 `-yaw(odom)` |

**关键含义**（（代码验证），`src/pb2025_sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp:136-144`）：
`gimbal_yaw_fake` 在 `odom` 里的 **yaw 恒为 0**（专门为全向底盘做的“假基座”）。
所以：

- costmap 里的 **footprint 永远不会随车体朝向旋转**，始终与 odom 轴对齐；
- 这对 **正方形 footprint（当前 0.6×0.6 m）没有影响**；
- 若以后换成非对称 footprint（例如带云台前伸的矩形），碰撞检查会失真 —— 那时请显式在描述里用对称包围盒，或改用真实 `gimbal_yaw` 并另做速度变换。

**排障**：`Can't update static costmap layer` / `Robot is out of bounds of the costmap` / `Timed out waiting for transform from gimbal_yaw_fake to odom` 的第一现场命令：

```bash
ros2 run tf2_ros tf2_echo odom gimbal_yaw_fake
ros2 run tf2_ros tf2_echo gimbal_yaw gimbal_yaw_fake
```

### 2.6 `rolling_window: true`（513 行）

| 项 | 值 |
|---|---|
| 作用 | 窗口跟随机器人；`false` 时窗口固定在 (origin_x, origin_y) |

局部图必须为 `true`。改成 `false` 且不设 `origin_x/origin_y`，机器人一旦离开初始区域就会报 `Robot is out of bounds of the costmap!`（`issues/001` 里出现过这条日志）。

### 2.7 `width: 8` / `height: 8`（514-515 行）

| 项 | 值 |
|---|---|
| 类型 | **int，单位 m**（写 `8.0` 会因类型不匹配报参数错误） |
| 作用 | 局部窗口边长；`rolling_window: true` 时窗口 = 2×4 m 的方形 |
| 默认 | 5 |

**容量校核**

- 8 m 窗口在 2.5 m/s 下只覆盖 **1.6 s** 的前方空间；
- 而 MPPI 的预测时域是 `time_steps 50 × model_dt 0.05 = 2.5 s`，理论上要看 6.25 m；
- 且 `obstacle_max_range: 3.0` 只让 3 m 内的障碍进入地图。

结论：**高速段（>1.5 m/s）时局部图是“短视”的**。两个改法（二选一或都做）：

- 把 `width/height` 提到 `10~12`（栅格数 200×200~240×240，CPU 线性增长，需实测）；
- 或把 `velocity_smoother.max_velocity` / MPPI `vx_max` 降到 1.5~2.0 m/s，并同步把 `obstacle_max_range` 提到 4.0。

### 2.8 `resolution: 0.05`（516 行）

| 项 | 值 |
|---|---|
| 类型 / 单位 | double / m/cell |
| 作用 | 栅格边长；同时是体素柱的 xy 分辨率 |
| 默认 | 0.1（Nav2 默认）/ 本文件 0.05 |

- 0.05 m 能分辨 5 cm 台阶，是 RM 场地的合理值。
- 代价：8 m 窗口下 25 600 格 × 16 层 = **409 600 个体素**，每周期全量 `reset`（`intensity_voxel_layer.cpp:updateBounds` 里每帧 `resetMaps()`）。
- 不建议低于 0.03：点云稀疏时反而会“点之间漏格”，MPPI 采样会在缝隙里穿模。

### 2.9 `robot_radius: 0.2`（517 行）与 `footprint: ...`（518 行）

| 项 | 值 |
|---|---|
| `robot_radius` | 0.2 m —— **被 footprint 覆盖，实际不起作用** |
| `footprint` | 正方形 0.6 × 0.6 m，中心在 `gimbal_yaw_fake` 原点 |

**（代码验证）**：`nav2_costmap_2d::Costmap2DROS` 解析参数时，只要 `footprint` 非空就使用它，`robot_radius` 仅在没有 footprint 时兜底。
所以想改机器人尺寸 **只能改 footprint**，改 `robot_radius` 不会有任何效果（这是最常见的“改了没反应”之一）。

当前 footprint 的派生量（后面算膨胀时会用到）：

| 量 | 计算 | 值 |
|---|---|---|
| 内切半径 inscribed | 边长/2 | **0.30 m** |
| 外接半径 circumscribed | 0.3×√2 | **0.4243 m** |
| 默认 padding | `footprint_padding`（未设置，Humble 默认 0.01 m，以 `ros2 param get` 实际值为准） | 0.31 / 0.434 m |

> 若实车实际尺寸大于 0.6×0.6 m，代价地图会“以为能过”。先量车再改这里，别用 `robot_radius` 试。

### 2.10 `plugins`（519 行）

```yaml
plugins: ["static_layer", "intensity_voxel_layer", "inflation_layer"]
```

- 列表**顺序 = 执行顺序**：先静态地图 → 再标动态障碍 → 最后膨胀。
- **膨胀必须最后**，否则动态障碍不会被膨胀（它只膨胀当前 master grid 里已有的代价）。
- `intensity_voxel_layer` 的 `combination_method: 1`（Maximum）保证它不会擦掉 `static_layer` 的代价。

---

## 3. `intensity_voxel_layer` 详解（520-546 行）

### 3.1 这个层到底做了什么（源码流程）

`intensity_voxel_layer.cpp` 的 `updateBounds()` 每帧做 5 件事：

1. `updateOrigin(robot_x - size/2, robot_y - size/2)` —— 滚动窗口原点按 `resolution` 对齐（**（代码验证）** `updateOrigin()` 用的是整格偏移量，所以原点只会跳整数格；RViz 里看起来是连续平移，属正常）；
2. `resetMaps()` —— **清空本层所有代价和体素**。意味着：
   - 本层 **没有记忆**，障碍物只要这一帧（或 buffer 里）没被观测到就立刻消失；
   - 不做 raytrace 清除（`clearing` / `raytrace_*` 参数对本层 **完全无效**）；
   - 地图上的“清除”靠 `terrain_analysis` 自己的 `clearingDis: 10.0`。
3. 取 **marking** 观测（`getMarkingObservations`），逐点做 4 道过滤（见 §3.4）；
4. 命中体素就写 `LETHAL_OBSTACLE (254)`，并 `touch()` 扩展更新区域；
5. `updateFootprint()` —— `footprint_clearing_enabled: true` 时把 **footprint 覆盖的格子强制置 0（FREE）**。

第 5 步的副作用：**车体正下方的障碍物永远不可见**（这是 Nav2 的标准行为，防自碰），代价是“顶着障碍物倒车”时近处不会报警。

### 3.2 层通用参数

| 参数 | 当前 | 默认 | 说明 / 调参 |
|------|------|------|-------------|
| `plugin` | `pb_nav2_costmap_2d::IntensityVoxelLayer` | — | 本仓库自定义层，继承 `nav2_costmap_2d::ObstacleLayer` |
| `enabled` | `true` | `true` | 可运行时热改（父类注册了动态参数回调）；置 `false` 后本层不再标记，但静态层/膨胀照旧。**注意本层自定义参数（`min/max_obstacle_intensity`、`z_*`、`origin_z`、`mark_threshold`）没有动态回调，只在 configure 时读取** |
| `combination_method` | `1` | `1` | **0 = Overwrite（覆盖），1 = Maximum（取最大）**（见 §5.1 注释错误） |
| `footprint_clearing_enabled` | `true` | `true` | 见 §3.1 第 5 步。调试“为什么脚下有障碍没显示”时先看它 |
| `publish_voxel_map` | `false` | `false` | 置 `true` 会发布 `~/voxel_grid`（`nav2_msgs/VoxelGrid`），可配合 nav2 的 VoxelGrid 显示/自写节点排查高度问题。**仅调试时开** |
| `track_unknown_space` | `true` | — | ⚠️ **本插件没有读取这个参数**（§5.4）。未知空间的处理看顶层 `local_costmap` 的 `track_unknown_space`（未设置 → `false`） |

### 3.3 体素几何参数

| 参数 | 当前 | 默认 | 说明 |
|------|------|------|------|
| `origin_z` | `0.0` | 16.0（插件默认值很怪，务必显式写） | 体素窗口的 **z 下边界**（odom 系） |
| `z_resolution` | `0.05` | 0.05 | 每个体素的高度 |
| `z_voxels` | `16` | 16 | 竖直方向层数。**硬上限 16，超过无效**（见下方⚠️） |
| `unknown_threshold` | `5` | 15 | 插件里 **声明了但从未使用**（§5.3） |

**有效竖直窗口 = `[origin_z, origin_z + z_voxels × z_resolution)` = [0.00, 0.80) m**

> ### ⚠️ `z_voxels` 硬上限 16（实测踩过）
>
> `nav2_voxel_grid::VoxelGrid` 的每个 xy 栅格只用 **1 个 `uint32_t`** 存整列：
> 低 16 位记“未知”、高 16 位记“已标记”，因此 `1 << z` 只能 shift 0~15。
>
> **（代码验证）** `VoxelGrid::resize()` 反汇编：
> ```asm
> cmp  $0x10,%r13d      ; size_z > 16 ?
> jbe  284b            ;  <=16 正常
> call rcuts_log       ;  "Error, this implementation can only support up to 16 z values (%d)"
> movl $0x10,0x8(%rbp) ;  内部 size_z_ 被强制改成 16
> ```
>
> 危险之处在于 **两个 `size_z_` 不一致**：
>
> | 位置 | 值 | 作用 |
> |---|---|---|
> | `IntensityVoxelLayer::size_z_` | 配置值（如 80） | `worldToMap3D()` 判 `mz < size_z_` → 0~79 **全放行** |
> | `voxel_grid_.size_z_` | **被 clamp 成 16** | `markVoxelInMap()` 判 `z >= size_z_` → **mz ≥ 16 静默 `return false`** |
>
> 后半句是 `RCLCPP_DEBUG`，**没有任何日志**，表现为“动态层突然一个障碍都没有”。
>
> 真实案例（2026-10-23）：把 `z_resolution` 改 0.01、`z_voxels` 改 80 想提高竖向精度 →
> 有效窗口从 0.8 m 掉到 **0.16 m**，而 `/terrain_map` 的点 z 中位数是 **0.594 m** → 动态障碍全部丢失。
> 日志里只有构造时一条：
> ```
> [voxel_grid]: Error, this implementation can only support up to 16 z values (80)
> ```
>
> **结论：`z_voxels` 只能 ≤ 16。想扩大竖直覆盖，唯一手段是增大 `z_resolution`。**

而代码里的越界处理（**（代码验证）** `intensity_voxel_layer.cpp:168-179`）：

```cpp
if (pz < origin_z_) {                      // 低于下边界 → 夹到最底层，仍然标记
  worldToMap3D(px, py, origin_z_, ...);
} else if (!worldToMap3D(px, py, pz, ...)) { // 高于上边界 → 直接丢弃
  continue;
}
```
→ **低于 0 m 的点不会丢**（被压到第 0 层），**高于 0.8 m 的点全部丢弃**。

> ⚠️ 这条约束在“爬坡 / 上台阶”时非常危险：`/terrain_map` 的 z 是 **odom 系绝对高度**，机器人爬升 0.8 m 后，所有点都 ≥ 0.8 m，**局部代价地图会瞬间全空**（车在坡上“看不见”障碍）。
> 详见 §4.1 与 §6 的处方。

### 3.4 障碍点过滤链（按代码执行顺序）

一个点必须 **连续通过 4 道过滤** 才会被标成 lethal：

| 顺序 | 过滤项 | 参数（当前值） | 坐标系 / 度量 | 备注 |
|------|--------|----------------|----------------|------|
| 1 | 高度 | `min_obstacle_height 0.03`、`max_obstacle_height 2.0` | **odom 绝对 z** | 只对地面有效；坡度/漂移下会失效 |
| 2 | 强度 | `min_obstacle_intensity 0.05`、`max_obstacle_intensity 1.8` | `intensity` = **相对局部地面高度** | **这才是可靠的“离地高度”过滤器** |
| 3 | 距离 | `obstacle_min_range 0.3`、`obstacle_max_range 3.0` | 到 `base_footprint` 的 **3D 距离** | `sq_dist <= min²` 与 `>= max²` 都被丢弃 |
| 4 | 体素边界 | `origin_z / z_voxels / z_resolution` | odom 绝对 z | §3.3 的窗口 |

#### 1) `min_obstacle_height: 0.03` / `max_obstacle_height: 2.0`

- **（代码验证）** 这两个是 **layer 级**参数（`intensity_voxel_layer` 直接读 `name_ + ".max_obstacle_height"`，`min_` 由父类 `ObstacleLayer` 读取）。
- 作用在 **odom 系绝对 z**：平地上等价于“离地 3 cm 以上”，但 odom z 有漂移或机器人爬坡时 **语义就变了**。
- 上限 2.0 m 实际不生效 —— 先被 §3.3 的 0.8 m 窗口截断。
- 建议：**不要靠它做地面过滤**，把它当“安全兜底”（例如 `min_obstacle_height: -0.5`），地面交给 `min_obstacle_intensity`。

#### 2) `min_obstacle_intensity: 0.05` / `max_obstacle_intensity: 1.8`

上游 `terrain_analysis` 的输出语义（**（代码验证）** `terrainAnalysis.cpp:562-569`）：

```cpp
float disZ = point.z - planarVoxelElev[...];     // 相对“局部地面平面”的高度
if (disZ >= 0 && disZ < vehicleHeight && ...) {  // vehicleHeight = 0.5
  terrainCloudElev->points[...].intensity = disZ;
```

所以：

| 事实 | 结论 |
|------|------|
| `intensity ∈ [0, vehicleHeight) = [0, 0.5)` | `max_obstacle_intensity: 1.8` **永远不会生效**（§5.2） |
| `intensity` 是相对地面高度，不受 odom z 漂移影响 | **调“什么算障碍物”应该调这个** |
| `min_obstacle_intensity: 0.05` | 离地 < 5 cm 的点被当成地面噪声丢弃 |

调参方向：

- **漏检低矮障碍**（5 cm 线缆、场地边缘、倒地标）：`0.05 → 0.02~0.03`；
  若仍漏，问题在上游：`terrain_analysis.vehicleHeight: 0.5`（>0.5 m 高的悬空物根本不会输出）与 `quantileZ: 0.75`（地面估计偏高会把矮物当地面）。
- **地面/草皮/坡面被当障碍**（局部图一片 254）：`0.05 → 0.08~0.12`，同时检查 `terrain_analysis.useSorting / considerDrop / minBlockPointNum`。

#### 3) `obstacle_max_range: 3.0` / `obstacle_min_range: 0.3`

- 都是 **到 `sensor_frame`（`base_footprint`）原点的 3D 欧氏距离**，与窗口尺寸无关。
- `3.0 m` 与 4 m 的半窗口是匹配的（0~3 m 有障碍，3~4 m 是空白）。**想让高速段看得更远，必须同时调大窗口和这里。**
- `0.3 m` 是 **盲区**：车头前方 30 cm 内的障碍会被直接滤掉。**（代码验证）** 过滤条件是 `sq_dist <= sq_obstacle_min_range → continue`。
  - 推荐降到 `0.1~0.15`（保守）或 `0.0`（完全依赖 footprint 清除）。
  - 注意副作用：贴车体噪点（车体自反射、地面近点）会开始进图；若出现“车被自己围住”，先查 `min_obstacle_intensity`，再回退这个值。

#### 4) 体素边界

见 §3.3。

### 3.5 `mark_threshold: 0`（527 行）

| 项 | 值 |
|---|---|
| 类型 / 默认 | int / 0 |
| 语义 | 一根体素柱里 **至少要有多少个不同高度的体素被命中**，该 xy 栅格才变成 lethal |

**（代码验证）** `nav2_voxel_grid/voxel_grid.hpp:99-116`：

```cpp
*col |= full_mask;                       // 记录命中
unsigned int marked_bits = *col >> 16;
return !bitsBelowThreshold(marked_bits, marked_threshold);
```
`bitsBelowThreshold(n, t)` 在 `popcount(n) > t` 时返回 `false`，取反后即 **命中数 > threshold 才标记**。

| 值 | 效果 | 适用 |
|----|------|------|
| `0`（当前） | 1 个点即致命 | 最灵敏，也最容易把噪点变障碍 |
| `1~2` | 需要 2~3 个不同高度的点 | 抑制单点噪点（草地、雨雾、玻璃反射） |
| `>= 3` | 只对“有厚度”的物体生效 | 会漏掉薄片（板、网）；**RM 场地不建议** |

> 注意：因为每帧 `resetMaps()`，`mark_threshold` 统计的是 **单帧内** 的体素柱，不是多帧累积。想要“多帧确认”应改用 `observation_persistence`（§3.6）。

### 3.6 `observation_sources` 与 `terrain_map` 观测源（536-546 行）

```yaml
observation_sources: terrain_map
terrain_map:
  data_type: PointCloud2
  topic: /terrain_map
  sensor_frame: base_footprint
  min_obstacle_height: -1.0
  max_obstacle_height: 1.5
  obstacle_max_range: 3.0
  obstacle_min_range: 0.3
```

| 参数 | 当前 | 默认 | 说明 |
|------|------|------|------|
| `data_type` | `PointCloud2` | LaserScan | 与 `/terrain_map` 匹配 |
| `topic` | `/terrain_map` | 源名 | `terrain_analysis` 发布，frame = **`odom`** |
| `sensor_frame` | `base_footprint` | `""`（用消息 header） | **只用来确定 `obs.origin_`**（算距离的起点）。设为 `base_footprint` 即“从车体中心量距离” |
| `min_obstacle_height` | `-1.0` | 0.0 | ⚠️ 见下方“无效参数”说明 |
| `max_obstacle_height` | `1.5` | 0.0 | ⚠️ 同上 |
| `obstacle_max_range` | `3.0` | 2.5 | 生效（见 §3.4-3） |
| `obstacle_min_range` | `0.3` | 0.0 | 生效（见 §3.4-3） |
| `observation_persistence` | **`0.1`**（2026-10-22 显式添加） | 0.0 | 见下方说明 |

**未写但影响行为的 source 参数（Humble 默认值）**

| 参数 | 默认 | 对本层的影响 |
|------|------|--------------|
| `expected_update_rate` | `0.0`（不限） | 数据断流时 `current_` 变 false，但本层仍会继续输出上一帧结果 |
| `marking` | `true` | 本层只消费 marking（`getMarkingObservations`） |
| `clearing` | `false` | **无效**：本层没有 raytrace 代码 |
| `raytrace_max_range` / `raytrace_min_range` | 3.0 / 0.0 | **无效**：同上 |
| `inf_is_valid` | `false` | 只对 LaserScan 的 `inf` 有效，与本层无关 |

#### `observation_persistence: 0.1`（两个代价地图均已添加）

**（代码验证）** 这是 Nav2 原生的观测源参数（`liblayers.so` 中有 `observation_persistence` 字符串），单位秒，`ObservationBuffer` 会保留最近 N 秒的点云；每个 `updateBounds()` 周期处理缓冲里的**全部**帧。

它对本层的含义与官方层不同：本层每帧 `resetMaps()`，所以障碍的“存活期”完全由缓冲决定。

| 取值 | 移动障碍表现 | 副作用 |
|---|---|---|
| `0`（原默认） | 一帧丢点就消失 → **闪烁** | 规划器可能在“空窗”重规划而看不到障碍 |
| **`0.1`（当前）** | 20 Hz 源 ≈ 2 帧，抗闪烁 | ghost 长度 ≈ 0.25 m（2.5 m/s），可忽略 |
| `0.3` | 更平滑 | ghost ≈ 0.75 m，全局图会避开障碍已离开的位置 |
| `>0.5` | 很“粘” | 明显拖影，移动障碍基本避不开 |

CPU 代价：每次 `updateBounds` 处理的点云帧数 = `persistence × 源频率`，当前为 2 帧（源实测 20 Hz）。
| `marking` | `true` | 本层只消费 marking（`getMarkingObservations`） |
| `clearing` | `false` | **无效**：本层没有 raytrace 代码 |
| `raytrace_max_range` / `raytrace_min_range` | 3.0 / 0.0 | **无效**：同上 |
| `inf_is_valid` | `false` | 只对 LaserScan 的 `inf` 有效，与本层无关 |

> ⚠️ **观测源里的 `min_obstacle_height / max_obstacle_height` 是“无效参数”**（代码验证）：
> Nav2 的 `Observation` 结构体只携带 `obstacle_*_range` / `raytrace_*_range` / `origin_` / `cloud_`，**没有高度字段**（`/opt/ros/humble/include/nav2_costmap_2d/nav2_costmap_2d/observation.hpp`）。
> 这两个值被传给 `ObservationBuffer` 后就不再参与任何过滤；真正的高度过滤在 `updateBounds()` 里用 **layer 级** 的 0.03 / 2.0 完成。
> 所以把源里的 `-1.0 / 1.5` 改成别的值 **不会有任何效果** —— 想放宽高度请改 §3.4-1 的 layer 级参数。

---

## 4. 当前配置的定量分析（调参前先看这三笔账）

### 4.1 ⚠️ 竖直窗口只有 0.8 m（最容易踩的坑）

```
窗口上边界 = origin_z + z_voxels × z_resolution
          = 0.0 + 16 × 0.05 = 0.80 m
```

- 平地：`/terrain_map` 点的 z ≈ 0（odom 原点在启动位置地面），`terrain_analysis` 又只放行 `z - vehicleZ ∈ (-0.35, +0.30)`（并随距离按 `disRatioZ: 0.08` 放宽）且 `disZ < vehicleHeight = 0.5` 的点，所以 0.8 m 窗口“够用”；
- **爬坡/上台阶/odom z 漂移 > 0.8 m**：所有点的 z 都落在窗口外 → **一个障碍都不会入图**，但日志里没有任何报错（只是静默丢点）。
- 这也是“上坡后局部图一片空、随后撞墙”的典型成因。

**处方（按侵入性从低到高）**

| 方案 | `origin_z` | `z_resolution` | `z_voxels` | 窗口 | 说明 |
|---|---|---|---|---|---|
| **现状（推荐）** | `0.0` | `0.05` | `16` | [0.0, 0.8) | 平地够用 |
| 要覆盖爬坡 | `-0.8` | `0.10` | `16` | [-0.8, 0.8) | 竖向精度 10 cm，对“柱状命中”够用 |
| 要覆盖 ±1 m | `-1.0` | `0.125` | `16` | [-1.0, 1.0) | |
| ~~要 1 cm 精度~~ | — | ~~0.01~~ | ~~80~~ | ~~[0, 0.16)~~ | ❌ **不可行**：`z_voxels>16` 被 clamp，窗口反而缩到 0.16 m |

> ⚠️ `z_voxels` 必须 ≤ 16（见 §3.3 的硬上限说明）。想扩大覆盖只能牺牲竖向分辨率。

> 注意 `origin_z` 变负后，低于它的点会被夹到最底层（仍会标记），不会丢。

### 4.2 障碍物的“有效高度”是 0.5 m，不是 2.0 m

上游 `terrain_analysis.vehicleHeight: 0.5` 决定：**只有离地 < 0.5 m 的点会被发布**。
所以“`max_obstacle_height: 2.0` 能看见高墙”是错的 —— 高墙只是靠 **墙脚 0~0.5 m 的那一圈点** 被标记。
这通常没问题（墙脚一定着地），但两类目标会整体漏检：

- 悬空物（桌沿、横幅、斜撑）
- 最低点高于 0.5 m 的物体

对策：调 `terrain_analysis.vehicleHeight`（会同时增大地面误检与计算量），或另加一路点云源（例如 `pointcloud_to_laserscan` 的 `/scan`）作为第二个 observation source。

### 4.3 膨胀半径 0.34 m 只比内切半径多 4 cm

Nav2 膨胀层的代价公式（`inflation_layer`）：

```
d <= inscribed(0.30)          → 253 (INSCRIBED_INFLATED_OBSTACLE)
inscribed < d <= inflation_radius(0.34)
                              → 252 · exp(-cost_scaling_factor · (d - inscribed))
                              = 252 · exp(-10 · (d - 0.30))
d > inflation_radius          → 0
```

代入当前值：

| 距障碍距离 d | 代价 | 含义 |
|---|---|---|
| ≤ 0.30 m | 253 | 内切，必撞（MPPI `CostCritic.critical_cost: 253` 在此触发惩罚） |
| 0.31 m | 228 | |
| 0.34 m | 169 | **梯度带终点** |
| > 0.34 m | 0 | 完全无代价 |

也就是说：**只有 4 cm 宽的“缓冲区”**。`cost_scaling_factor: 10` 又让衰减极快，实际效果接近“硬边界”。
这与配置注释里“防止贴障行走”的意图相反 —— 机器人会贴着障碍物走，只在 30 cm 内才被强惩罚。

**处方（对照 sim 配置 `src/pb2025_sentry_sim/config/nav2.yaml`：`0.65 / 3.0`）**

| 目标 | `inflation_radius` | `cost_scaling_factor` | 说明 |
|------|--------------------|-----------------------|------|
| 保守绕障（推荐先试） | `0.55~0.60` | `4~6` | 梯度带宽 25~30 cm，MPPI 有平滑排斥力 |
| 窄道通过 | `0.40~0.45` | `6~8` | 勉强能过 0.9 m 缝 |
| 当前值 | `0.34` | `10` | 几乎无梯度，只在撞前 30 cm 刹车 |

辅助参数（未写，默认 `false`，一般保持）：

- `inflate_unknown`：把未知区域也膨胀（本配置顶层 `track_unknown_space` 为 `false`，未知区已被当自由，开了也基本无感）
- `inflate_around_unknown`：同上

> 与 MPPI 的联动：`CostCritic.critical_cost: 253` 意味着 **只有 ≥253 的格子才算“碰撞”**。膨胀半径过小 + 因子过大时，costmap 里几乎没有 100~200 的“软代价”，CostCritic 的梯度引导基本失效，只剩 `ObstaclesCritic` 的硬惩罚 —— 表现就是“贴障后才猛打方向”。

### 4.4 CPU 与延迟预算

| 项 | 数值 | 备注 |
|---|---|---|
| 栅格数 | 160 × 160 = 25 600 | |
| 体素数 | 25 600 × 16 = 409 600 | `resetMaps()` 每周期清空 |
| 点云规模 | `/terrain_map` ≈ 数千~数万点 | `terrain_analysis.scanVoxelSize: 0.03` 已降采样 |
| 更新率 | 25 Hz | 与 MPPI 同进程 |

若 `top` 里 `controller_server` 长期 > 100% 单核：先把 `update_frequency` 降到 15~20，再看 `terrain_map` 的点数（`ros2 topic hz` + `ros2 topic bw`）。`resolution` 会同时改变精度与开销（格数按边长平方变化），不是首选旋钮。

---

## 5. 配置文件里需要修正的注释 / 无效项

> ✅ **2026-10-21 已修正**：下面 5.1~5.5 涉及的注释已在 `nav2_params.yaml` 中就地改写，改动行以
> `[注释修正]`（原文有误）或 `[注释补充]`（原文缺失关键语义）标记，可直接定位：
>
> ```bash
> grep -n '# \[注释' src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml
> ```
>
> 逐行清单见 §5.7。**参数值一个都没动**（`yaml.safe_load` 前后完全一致）。

### 5.1 `combination_method` 注释写反了（526 行）

```yaml
combination_method: 1   # 注释：0-最大值（Max）、1-覆盖（Overwrite）、2-平均值（Average）
```

实际实现（**（代码验证）**，`nav2_costmap_2d::ObstacleLayer::updateCosts` 反汇编 + 源码语义）：

| 值 | 实际行为 | 本配置 |
|---|---|---|
| `0` | **Overwrite**（`updateWithOverwrite`，本层值直接覆盖 master） | |
| `1` | **Maximum**（`updateWithMax`，取两者较大值） | ✅ 当前值 |
| 其他 | 不组合（等于关闭本层效果） | |

- 不存在 “2 = Average”。若真按注释改成 `0`，动态层会把 `static_layer` 的代价整片覆盖成 FREE，静态墙体会消失。
- **保持 `1` 不变，把注释改掉**。

### 5.2 `max_obstacle_intensity` 的注释描述的是最小值（535 行）

```yaml
max_obstacle_intensity: 1.8   # 注释：“...最小强度值。机器人将忽略强度小于此值的点...”
```

- 注释描述的是 `min_obstacle_intensity` 的作用。
- 且由于上游 `intensity < vehicleHeight = 0.5`，`1.8` **永远不生效**（§4.2）。想过滤“太高的悬空物”应调 `terrain_analysis`，不是这里。

### 5.3 `unknown_threshold` 在本插件里是死参数（529 行）

- 自定义层 `onInitialize()` 里 `unknown_threshold_ = declare_parameter(...)` 之后 **没有任何读取点**（`intensity_voxel_layer.cpp:49`）。
- 保留它没有副作用，但**改它不会有任何变化**，不要在这里浪费调参时间。

### 5.4 层内 `track_unknown_space: true` 不生效（523 行）

- 本插件没有读取 `track_unknown_space`；它只由顶层 `LayeredCostmap` 使用。
- 本文件 **顶层没有设置** `track_unknown_space` → 取默认 `false` → 未被观测的格子按 **FREE(0)** 处理。
- 如果希望“没看过的地方”在局部图中保持未知（更保守），要在 `local_costmap.local_costmap.ros__parameters` 下 **顶层** 加 `track_unknown_space: true`，而不是加在层里。

### 5.5 `robot_radius` 被 footprint 覆盖（517 行）

见 §2.9：不是错误，但容易误导。

### 5.6 `behavior_server` 里的 `local_costmap_topic` 系列键在 Humble 中不生效（653-656 行）

```yaml
behavior_server:
  ros__parameters:
    local_costmap_topic: local_costmap/costmap_raw      # Humble 中未被读取
    global_costmap_topic: global_costmap/costmap_raw    # Humble 中未被读取
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
```

**（代码验证）** Humble 的 `behavior_server` 只声明 `costmap_topic` / `footprint_topic`（`/opt/ros/humble/lib/libbehavior_server_core.so` 中只有这两个字符串，不存在 `local_costmap_topic`）。
这些键是 Iron/Jazzy 时代的写法，在 Humble 里被自动声明但 **不会被读取** —— 想改 `behavior_server` 用的地图，应写 `costmap_topic` / `footprint_topic`（默认即 `local_costmap/costmap_raw` / `local_costmap/published_footprint`，所以当前默认行为仍是对的）。
这也是为什么 `publish_frequency` 会直接影响恢复行为（`spin` / `backup`）的反应速度。

### 5.7 本次注释修改清单（2026-10-21）

| 行 | 参数 | 原注释的问题 | 修改后写的是 |
|---|------|--------------|--------------|
| 517 | `robot_radius` | 未说明它已被 footprint 覆盖 | 有 footprint 时被忽略，改尺寸请改 footprint |
| 518 | `footprint` | 无任何说明 | 0.6×0.6 m，内切 0.30 / 外接 0.424 |
| 523 | `track_unknown_space` | 写在层里，易被当成生效参数 | 本插件不读取，只有顶层 `LayeredCostmap` 使用 |
| 526 | `combination_method` | **0/1 含义写反**，且虚构了 “2=Average” | 0=Overwrite、1=Maximum，不存在 Average |
| 527 | `mark_threshold` | 无说明 | 单帧内体素柱命中层数 > 此值才标记；0=单点致命 |
| 528 | `origin_z` | 只说“通常设为 0”，没说是窗口下边界 | 窗口 = `[origin_z, origin_z + z_voxels×z_resolution)` = [0, 0.8) m |
| 529 | `unknown_threshold` | 被描述成“强度阈值” | 本插件声明后未使用（死参数） |
| 530 | `z_resolution` | 没提它决定竖直覆盖高度 | 覆盖高度 = `z_voxels × z_resolution` |
| 531 | `z_voxels` | **复制了 `z_resolution` 的文字** | 竖直层数；覆盖高度公式；超窗静默丢点 |
| 532 | `min_obstacle_height` | 只写“最低高度”，未说明是 layer 级 / odom z | layer 级、作用于 odom 绝对 z；地面过滤交给强度 |
| 533 | `max_obstacle_height` | 声称 2.0 m 生效 | 实际被 0.8 m 体素窗口截断，此值不生效 |
| 534 | `min_obstacle_intensity` | 正确但过于笼统 | 明确 `intensity` = 相对地面高度 ∈[0,0.5) |
| 535 | `max_obstacle_intensity` | **描述的却是“最小强度”** | 强度上限；上游恒 <0.5，此值不生效 |
| 543 | 源 `min_obstacle_height` | 声称会过滤高度 | 观测源级高度在 Nav2 中不参与过滤，改这里无效 |
| 544 | 源 `max_obstacle_height` | 同上 | 同上 |
| 545 | `obstacle_max_range` | 只写“最大距离” | 到 `base_footprint` 的 3D 距离上限，需与窗口匹配 |
| 546 | `obstacle_min_range` | 语气像“优化项” | 0.3 m 内直接丢弃 = 碰撞盲区，可降到 0.1~0.15 |
| 549 | `map_subscribe_transient_local` | 无说明 | 以 transient_local 订阅 `/map`，QoS 不匹配会一直报 no map received |
| 552-554 | `cost_scaling_factor` | **调参方向写反**（“增大→更远离障碍”） | 越大衰减越快、梯度带越窄（更贴障）；越小越保守；附公式与 MPPI 联动 |
| 555 | `inflation_radius` | 写“≥ 外接半径+余量”，但取值 0.34 不满足 | 指出当前只比内切 0.30 多 4 cm，注释与取值矛盾 |

> 未改动：`global_costmap` 段（558-605 行）里存在**完全相同**的注释错误（577 行 `combination_method`、578 行 `mark_threshold`、583-586 行高度/强度注释），本次按“只改 local”的范围没有触碰，需要时可同法处理。
> 也未改动 `behavior_server` 的 `local_costmap_topic` 等键（§5.6）—— 那是键名问题，改了会改变行为，不属于“注释修正”。

---

## 6. 按症状调参手册

| 症状 / 现象 | 最可能的原因 | 建议动作（按优先级） |
|-------------|--------------|----------------------|
| 局部图 **一片空**，但雷达明明看到墙 | ① **`z_voxels > 16` 被 clamp，窗口骤减**（最常见）② 爬坡/odom z 超出窗口 ③ `intensity` 被 `min_obstacle_intensity` 滤光 ④ `/terrain_map` 没数据 | ① `grep "16 z values" ~/.ros/log/*/controller_server*.log`；改回 `z_voxels: 16`；② 调 `origin_z` + 增大 `z_resolution`；③ 临时 `min_obstacle_intensity: 0.0` 验证；④ `ros2 topic hz /terrain_map` |
| **上坡后开始看不见障碍** | §4.1 竖直窗口 | 同上，优先把窗口下移（`origin_z` 负值）+ 加层数 |
| 低矮障碍（<5 cm）不避让 | `min_obstacle_intensity 0.05` / 上游 `vehicleHeight`、`quantileZ` | `0.05 → 0.02`；必要时调 `terrain_analysis` |
| 地面/草/坡被当障碍，图上一片红 | `min_obstacle_intensity` 太低 / 上游地面估计 | `0.05 → 0.08~0.12`；检查 `terrain_analysis.useSorting`、`minBlockPointNum` |
| 障碍 **闪烁**、时有时无 | 每帧 `resetMaps()` + `observation_persistence: 0` + `mark_threshold: 0` | 已设 `observation_persistence: 0.1`；仍闪可试 0.2（别超过 0.3，否则拖影） |
| 贴障擦边、拐角蹭、靠墙走 | 膨胀带宽只有 4 cm（§4.3） | `inflation_radius: 0.55`、`cost_scaling_factor: 5` |
| 窄缝（<1 m）过不去 | 膨胀太大 | 回退到 `0.40~0.45`，并确认 footprint 没写大 |
| 近处（<30 cm）不报警、倒车撞 | `obstacle_min_range: 0.3` 盲区 | `0.1~0.15`；同时确认 `footprint_clearing_enabled` 的副作用 |
| 高速段“冲过头” / 看到障碍太晚 | 8 m 窗 + 3 m 探测 vs 2.5 m/s | `width/height: 10~12`、`obstacle_max_range: 4.0`，或降速 |
| 局部图与全局图 **冲突**、反复横跳 | local 里也放了 `static_layer`，SLAM 建图时 `/map` 在变 | 定位模式保留；**纯 SLAM 模式**可临时移除 local 的 `static_layer`，只保留动态层 |
| `Can't update static costmap layer, no map received` | local 里配了 `static_layer`，但 `/map` 没来 | 确认 `map_server`/`slam_toolbox` 在跑；核对 `/map` 的 QoS 是否为 transient_local（与 `map_subscribe_transient_local: True` 匹配） |
| `Robot is out of bounds of the costmap` | `rolling_window: false` 或 TF 断裂 | 见 §2.6 / §2.5 的 TF 命令 |
| `controller_server` CPU 打满 | 25 Hz + 1200 采样 MPPI 同进程 | `update_frequency: 15~20`；或降 `batch_size` |

---

## 7. 调参工作流（建议照抄）

1. **先证伪**：`ros2 topic hz /terrain_map`、`ros2 run tf2_ros tf2_echo odom gimbal_yaw_fake`。
   数据/TF 不通时，改任何参数都没意义。
2. **单变量**：一次只改 1~2 个参数。Humble 中 `enabled`、`inflation_radius`、`cost_scaling_factor` 等层参数支持热改（`ros2 param set` 会返回 `Successful` 与否）；其余参数（含本插件自定义的 `min_obstacle_intensity`、`z_voxels`、`origin_z`、`mark_threshold`）只在 configure 时读取，**必须重启 `controller_server`**。
3. **可视化**：RViz 加 `Costmap2D`（topic `local_costmap/costmap`）+ `PointCloud2`（`/terrain_map`，Color Transformer = Intensity）。
   两者叠在一起看“哪些点进了图、哪些被滤掉了”，比看日志快 10 倍。
4. **量化**：用 `ros2 topic echo /local_costmap/costmap --once` 或截图对比，记录“同一场景改前/改后”。
5. **回填本文**：把最终采用的值写进 §8 速查表的“备注”列，避免下一个人重新踩。

---

## 8. 参数速查表

| 参数 | 当前值 | 单位 | 建议范围 | 调大 → | 调小 → |
|------|--------|------|----------|--------|--------|
| `update_frequency` | 25.0 | Hz | 15~25 | CPU ↑，响应 ↑ | CPU ↓，控制用到旧图 |
| `publish_frequency` | 8.0 | Hz | 5~10 | 可视化/RViz 顺滑；`behavior_server` 延迟 ↓ | 带宽 ↓，恢复行为迟钝 |
| `width` / `height` | 8 / 8 | m | 8~12 | 视野 ↑，CPU ↑ | 视野 ↓ |
| `resolution` | 0.05 | m | 0.05~0.1 | 精度 ↓、CPU ↓ | 精度 ↑、CPU ↑ |
| `inflation_radius` | 0.34 | m | 0.40~0.60 | 更保守、缝更难走 | 更贴障 |
| `cost_scaling_factor` | 10.0 | — | 4~8 | 梯度更陡、更贴障 | 梯度更缓、更保守 |
| `combination_method` | 1 | — | 1 | — | 0 = Overwrite（会擦静态层） |
| `origin_z` | 0.0 | m | -1.0~-0.5 | 窗口上移（低点被压到最底层，仍会标记） | 窗口下移（上边界同步降低，需配合 `z_voxels`） |
| `z_voxels` | 16 | 个 | **≤ 16（硬上限）** | **无效，会被 clamp 成 16** | 覆盖 ↓ |
| `z_resolution` | 0.05 | m | 0.05~0.125 | 覆盖 ↑、竖向精度 ↓（**扩大覆盖只能走这条路**） | 覆盖 ↓、竖向精度 ↑ |
| `mark_threshold` | 0 | 个 | 0~2 | 抗噪 ↑，薄障碍漏检 ↑ | 更灵敏 |
| `min_obstacle_height` | 0.03 | m(odom) | -0.5~0.05 | 更严格的地面过滤（但坡上失效） | 更宽松 |
| `max_obstacle_height` | 2.0 | m(odom) | 保留（实际被体素窗口约束） | — | — |
| `min_obstacle_intensity` | 0.05 | m(相对地面) | 0.02~0.12 | 抗地面噪点 ↑，矮障碍漏检 ↑ | 更灵敏 |
| `max_obstacle_intensity` | 1.8 | m(相对地面) | 保留（上游 ≤0.5） | — | — |
| `obstacle_max_range` | 3.0 | m(3D) | 3.0~4.0 | 看得远，CPU ↑ | 更近 |
| `obstacle_min_range` | 0.3 | m(3D) | 0.1~0.2 | 近距盲区 ↑ | 盲区 ↓，自噪点 ↑ |
| `observation_persistence` | 0.1 | s | 0.1~0.2 | 障碍更“粘”、抗闪烁；>0.3 出现拖影 | 更实时，但会闪烁 |
| `footprint_clearing_enabled` | true | — | true | — | false（脚下也会标障碍，一般别关） |

---

## 9. 快速验证命令

```bash
# 1) 参数是否真的被读到（Humble 里 local_costmap 是 controller_server 内部节点，
#    节点全名形如 /local_costmap/local_costmap；带 namespace 时为 /<ns>/local_costmap/local_costmap）
ros2 node list | grep -i costmap
ros2 param list /local_costmap/local_costmap | head -40
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap intensity_voxel_layer.z_voxels

# 2) 数据与频率
ros2 topic hz /terrain_map
ros2 topic echo /terrain_map --once --field header.frame_id
ros2 topic hz /local_costmap/costmap

# 3) TF（局部图取不到位姿时必查）
ros2 run tf2_ros tf2_echo odom gimbal_yaw_fake
ros2 run tf2_ros tf2_echo base_footprint odom

# 4) 运行时临时试参（热改：enabled / inflation_radius / cost_scaling_factor，以 param set 的返回为准）
#    自定义层参数（min_obstacle_intensity、z_voxels、origin_z、mark_threshold）热改无效，需重启 controller_server
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.55
ros2 param set /local_costmap/local_costmap intensity_voxel_layer.enabled false

# 5) 看体素（先把 publish_voxel_map 置 true）
ros2 topic hz /local_costmap/voxel_grid
```

> 当前 `install/` 是 `--symlink-install`（`install/.../nav2_params.yaml` 是软链），**改完 YAML 不需要 colcon build，重启 launcher 即可生效**。
> 但注意：插件源码（`pb_nav2_plugins`）改了就必须 rebuild。

---

## 10. 相关文件

- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:505-556`（本文主体）
- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:395-504`（MPPI：`CostCritic` / `ObstaclesCritic` 与膨胀的联动）
- `src/pb2025_sentry_nav/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp`（本层全部行为）
- `src/pb2025_sentry_nav/pb_nav2_plugins/include/pb_nav2_plugins/layers/intensity_voxel_layer.hpp`
- `src/terrain_analysis/src/terrainAnalysis.cpp:540-640`（`/terrain_map` 的 `z` 与 `intensity` 语义）
- `src/pb2025_sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp:136-144`（`gimbal_yaw_fake` 的 yaw 恒 0）
- `src/pb2025_sentry_sim/config/nav2.yaml:76-104`（对照：sim 的 `0.65 / 3.0` 膨胀与 `obstacle_layer` 写法）
- `issues/001-dual-lidar-timestamp-base-jump.md`（`local_costmap` 报错的下游表现）

---

## 处理记录

| 日期 | 处理人 | 改动 / 结论 |
|------|--------|-------------|
| 2026-10-21 | — | 建立文档：逐参数解释 + 源码核对 + 3 处注释错误（`combination_method`、`max_obstacle_intensity`、`unknown_threshold`）+ 竖直窗口 0.8 m 陷阱 |
| 2026-10-21 | — | 按 §5.7 就地修正 `local_costmap` 段 22 行注释（`[注释修正]` / `[注释补充]` 标记）；参数值未改，`yaml.safe_load` 前后完全一致 |
| 2026-10-22 | — | 两个代价地图的观测源均新增 `observation_persistence: 0.1`（抗移动障碍闪烁），见 §3.6 |
| 2026-10-23 | — | **修正本文错误结论**：`z_voxels` 硬上限 16（`VoxelGrid` 单 uint32 列），此前 §4.1 推荐的 24/32/36 全部无效。现场踩坑：`z_resolution 0.01 + z_voxels 80` → 窗口被 clamp 到 0.16 m → 局部动态层全空。已改回 `0.05 / 16` |
