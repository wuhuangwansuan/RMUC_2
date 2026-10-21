# 007. 🔴 导航 ⇄ 决策数据流审计：`/cmd_vel` 直发链路断开、建图门控未覆盖实际链路

- **状态**：🔴 待修（F1 为功能性断链，其余为待确认/降级）
- **发现日期**：2026-10-20
- **影响面**：功能失效（未开赛停车指令无效）+ 行为不确定（默认树切换、战术决策降级）
- **审计范围**：`pb2025_sentry_nav`（导航）、`pb2025_sentry_behavior`（决策）、`rm_serial_driver`（底盘/裁判）、`pb2025_sentry_sim`（仿真）
- **相关文件**：
  - `src/pb2025_sentry_behavior/behavior_trees/rmul_nuc_2026.xml:59`
  - `src/rm_serial_driver/src/rm_serial_driver.cpp:69`
  - `src/pb2025_sentry_nav/sensor_scan_generation/src/sensor_scan_generation.cpp:53`
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/slam_launch.py:130`
  - `src/pb2025_sentry_behavior/params/sentry_behavior.yaml:63`
- **关联问题**：[005](./005-small-gicp-registration-blocks-tf-publish.md)、[006](./006-coarse-registration-full-map-fpfh.md)

---

## 审计方法

```bash
# 1. 列出所有相关话题的订阅者
grep -rn "create_subscription<geometry_msgs::msg::Twist>\|create_subscription<Twist>" --include="*.cpp" src

# 2. 追踪 launch 中的 remap 关系
grep -n "remappings" -A4 src/pb2025_sentry_nav/pb2025_nav_bringup/launch/navigation_launch.py

# 3. 追踪裁判话题的发布者
grep -rn "create_publisher<" --include="*.cpp" src/rm_serial_driver/src/rm_serial_driver.cpp

# 4. 追踪建图点云来源
grep -rn "sensor_scan" src
grep -n "cloud_in\|obstacle_scan" src/pb2025_sentry_nav/pb2025_nav_bringup/launch/slam_launch.py
```

---

## 数据流总览

### 链路 A：决策 → 导航 ✅

```
pb2025_sentry_behavior_client
  └─ action "pb2025_sentry_behavior"（参数 target_tree）
       └─ BT: SendNav2Goal / TacticalNavigateToPose
            └─ action "navigate_to_pose"（nav2 bt_navigator）
                 goal: frame=map，格式 "x;y;yaw"
```

依赖 TF（现已贯通）：
`map→odom`（small_gicp）→ `odom→base_footprint`（sensor_scan_generation）→
`base_footprint→…→gimbal_yaw`（robot_state_publisher）→ `gimbal_yaw→gimbal_yaw_fake`（fake_vel_transform）

### 链路 B：导航 → 底盘 ✅

```
controller_server  cmd_vel  ──remap──▶  cmd_vel_controller
velocity_smoother  cmd_vel ◀── cmd_vel_controller
                   cmd_vel_smoothed ──remap──▶ cmd_vel_nav2_result
fake_vel_transform cmd_vel_nav2_result ─▶ cmd_vel_base_real_yaw
rm_serial_driver   订阅 /cmd_vel_base_real_yaw → 下位机
```

### 链路 C：决策 → 底盘（直发速度）❌

```
BT PublishTwist(topic_name="/cmd_vel") ─▶ /cmd_vel ─▶ （无订阅者）
```

### 链路 D：裁判系统 → 决策 / 导航 ⚠️

```
rm_serial_driver
  ├─ referee/game_status      ✓
  ├─ referee/robot_status     ✓
  ├─ referee/rfid_status      ✓
  └─ referee/event_data       ✓
        ├──▶ behavior server（BT 黑板）
        ├──▶ sensor_scan_generation（start_slam_ 门控）
        └──▶ tactical_scorer_node
     referee/ground_robot_position  ✗ 无发布者
     referee/all_robot_hp           ✗ 无发布者（仅仿真补发）
     referee/buff                   ✗ 无发布者
```

### 链路 E：导航 → 决策（反馈）✅

```
/global_costmap/costmap        (transient_local) ─▶ behavior "nav_globalCostmap"
/map                           (transient_local) ─▶ behavior "tactical_map"
/mapping_bootstrap_finished    (transient_local) ─▶ mapping_mission_manager ─▶ behavior
/tactical_decision             (QoS 10)          ─▶ tactical_scorer_node ─▶ behavior
```

---

## 发现清单

| # | 级别 | 问题 | 后果 |
|---|------|------|------|
| F1 | 🔴 | 决策层 `PublishTwist` 发往 `/cmd_vel`，真机无订阅者 | 未开赛“停车”指令无效；dev/2025 树直发速度无效 |
| F2 | 🟠 | `sensor_scan` 无订阅者，建图实际走 `terrain_map → obstacle_scan` 且未门控 | “比赛前不建图”的门控不成立 |
| F3 | 🟠 | 修复 TF 后 nav2 在比赛前即完成激活 | 决策层若门控失效可能提前运动 |
| F4 | 🟡 | `ground_robot_position` / `all_robot_hp` / `buff` 无发布者 | 战术决策的队友信息分支永久不可用 |
| F5 | 🟡 | 默认 `target_tree` 已改为 `sentry_tactical`，与历史日志不一致 | 下次启动会静默切换行为树 |
| F6 | 🟡 | `SendNav2Goal` 的 `action_name` 硬编码，与 `navigation_action_name` 参数脱节 | 改参数对旧节点无效 |

---

## F1 🔴 `/cmd_vel` 直发链路断开

### 现象

行为树中的停车/直发速度动作发布到 `/cmd_vel`，真机上没有任何节点订阅。

### 证据

树内话题（`rmul_nuc_2026.xml:59-63`）：

```xml
<PublishTwist v_y="0.0" v_yaw="0.0" v_x="0.0" duration="0.0"
              topic_name="/cmd_vel"/>
```

该节点位于 `rmul_nuc_2026_check_game_start` 的 `Fallback` 分支：
**未满足开赛条件时**执行 `PublishStatus(3)` → `PublishTwist(0,0,0)`，本意是让机器人保持静止。

订阅者检索结果：

| 订阅者 | 话题 | 场景 |
|---|---|---|
| `rm_serial_driver.cpp:69-70` | `/cmd_vel_base_real_yaw` | 真机底盘 |
| `fake_vel_transform.cpp:56` | `cmd_vel_nav2_result`（参数） | 导航输出 |
| `pb2025_sentry_sim/scripts/sim_adapter.py:41` | `cmd_vel` | **仅仿真** |
| `pb2025_sentry_behavior/test/*` | `/cmd_vel` | 单元测试 |
| `decision_test.launch.py:48` | 将 `/cmd_vel` remap 到 `/safety_stop` | **仅仿真** |

真机 launch（`rm_navigation_reality_launch.py` → `pb2025_sentry_behavior_launch.py`）**没有** `/cmd_vel` 的 remap。

### 影响

- 未开赛/裁判数据异常时的停车动作是空操作；
- `dev_rm.xml`、`rmul_2025.xml` 中作为主运动的 `PublishTwist` 同样无效；
- 仿真链路正常，因此仿真测试无法暴露该问题。

### 建议

- 真机 launch 增加 remap：`('/cmd_vel', '/cmd_vel_base_real_yaw')`；或
- 树里把 `topic_name` 改为 `/cmd_vel_base_real_yaw`。
- 全零速度直接映射是安全的；若将来用于非零速度，需确认是否应经过 `fake_vel_transform` 的 yaw 变换。

---

## F2 🟠 建图门控未覆盖实际链路

### 现象

`sensor_scan` 在仓库内**没有任何订阅者**（仅 RViz 配置引用），
而 slam_toolbox 实际使用的是另一条未经门控的链路。

### 证据

- 发布者：`sensor_scan_generation.cpp:53`（`pub_laser_cloud_` 发布 `sensor_scan`）
- 订阅者：`grep -rn "sensor_scan" src` → 只有 `rviz/nav2_default_view.rviz:345`；
  以及 `FAST_LIO/launch/mapping.launch.py` 里的**节点名**，不是话题订阅
- 实际建图链路：
  - `terrain_analysis.cpp:255,258` 订阅 `lidar_odometry` / `registered_scan`（**全文无 `game_status` 门控**）
  - 发布 `terrain_map`
  - `slam_launch.py:130-131`：`cloud_in ← terrain_map`，`scan → obstacle_scan`
  - `nav2_params.yaml:273`：`slam_toolbox.scan_topic: obstacle_scan`

### 影响

“比赛开始前不建图”的控制点放在 `sensor_scan` 上，但当前 `slam:=True` 流程不消费该话题，
因此门控对实际建图不起作用；比赛前只要启动 slam 模式就会开始建图。

### 建议

- 若确实需要在开赛前阻止建图，应门控 `terrain_map` 的输出（或在 `pointcloud_to_laserscan` 前加门控），
  而不是 `sensor_scan`；
- 或确认现场建图使用的是另一套订阅 `sensor_scan` 的配置。

---

## F3 🟠 修复 TF 后 nav2 在比赛前即激活

### 现象

[005](./005-small-gicp-registration-blocks-tf-publish.md) 修复后，`odom→base_footprint` 不再受比赛门控，
`lifecycle_manager_navigation` 在开赛前即可把 `controller_server` / `planner_server` / `bt_navigator`
全部激活（修复前是永久卡在 `activating`）。

### 影响

- `navigate_to_pose` action server 在开赛前就可用；
- 当前 BT 主链路有 `IsGameStatus(expected_game_progress=4)` 在前（`rmul_nuc_2026.xml` 的 `check_game_start`），
  正常情况下不会误发目标；
- 但如果决策层门控被绕过/参数改动（如 F5），存在提前运动的可能。

### 建议

在决策层增加显式保护：比赛未开始时主动 `cancel` 所有导航目标，并保持 `sensor_scan`/建图门控语义不变。

---

## F4 🟡 三个裁判话题无发布者

### 证据

订阅方：

- `pb2025_sentry_behavior_server.cpp:206` → `referee/all_robot_hp`
- `pb2025_sentry_behavior_server.cpp:208-209` → `referee/ground_robot_position`
- `pb2025_sentry_behavior_server.cpp:212` → `referee/buff`
- `tactical_scorer_node.cpp:153-154` → `referee/ground_robot_position`

发布方（`rm_serial_driver.cpp:36-43`）只有：

```
referee/event_data
referee/game_status
referee/robot_status
referee/rfid_status
/rm_serial_driver/received_robot_mode
```

仅仿真脚本 `sentry_referee_simulator.py:117` 发布 `all_robot_hp`。

### 影响

`tactical_scorer_node` 中：

```cpp
const bool hp_fresh        = all_robot_hp_ && isFresh(...);
const bool positions_fresh = ground_robot_position_ && isFresh(...);
```

恒为 `false` → `CalculateTeammateCoverGoal`、队友血量相关的决策分支不可用，
战术树会长期落在兜底动作（如 `tactical_hold_safe`）。

### 建议

- `rm_serial_driver` 补充这三个话题的发布；或
- `tactical_scorer_node` 在数据缺失时输出明确日志（当前是静默降级）。

---

## F5 🟡 默认行为树与历史日志不一致

### 证据

- `params/sentry_behavior.yaml:63`：`target_tree: sentry_tactical`
- `rm_navigation_reality_launch.py:220-224` 只透传 `namespace` / `use_sim_time` / `mapping_bootstrap_enabled`，
  **不覆盖** `target_tree`
- 2026-10-20 19:26 的实机日志：`Sending goal to execute Behavior Tree: rmul_nuc_2026`

### 影响

下一次启动将执行 `sentry_tactical` 而不是日志中的 `rmul_nuc_2026`。
而 `sentry_tactical` 依赖 F4 中缺失的裁判数据，实际行为会与预期不同。

### 建议

明确现场使用哪棵树，并在 launch 中显式传参，避免依赖 yaml 默认值。

---

## F6 🟡 `action_name` 双轨

### 证据

- 旧节点：`send_nav2_goal.cpp:150` 提供 `action_name` 端口，默认 `"navigate_to_pose"`；
  树里每条 `SendNav2Goal` 都显式写死 `action_name="navigate_to_pose"`
- 新参数：`pb2025_sentry_behavior_server.cpp:71-74` 读取 `navigation_action_name` 写入黑板；
  只有 `TacticalNavigateToPose` 使用 `{@navigation_action_name}`

### 影响

修改 `navigation_action_name` 参数不会影响旧树中的 `SendNav2Goal`，容易出现“参数改了但没生效”。

### 建议

统一为 `action_name="{@navigation_action_name}"`，或在文档中明确该参数只对新节点生效。

---

## 已验证正确的部分

| 检查项 | 结果 |
|---|---|
| `map→odom→base_footprint→gimbal_yaw→gimbal_yaw_fake` TF 链 | ✅ 完整 |
| `controller_server → velocity_smoother → fake_vel_transform → 串口` 速度链 | ✅ remap 一致 |
| `/map`、`/global_costmap/costmap`、`/mapping_bootstrap_finished` QoS | ✅ 发布/订阅均为 `transient_local().reliable()` |
| `/tactical_decision` 类型与 QoS | ✅ 均为 `TacticalDecision`、QoS 10 |
| `sentry_behavior.yaml` 中 `use_sim_time: true` | ✅ 会被 launch 的 `param_rewrites` 覆盖为实参，无影响 |

---

## 本次会话的改动边界

为避免混淆，明确记录：**本审计对应的会话只改动了以下文件**，其余改动（`pb2025_sentry_behavior/**`、
`rm_navigation_reality_launch.py`、`small_gicp_relocalization` 的 P2-2 代码、新增仿真/接口包）
均为工作区既有改动，本会话未编辑。

| 文件 | 改动 |
|---|---|
| `src/pb2025_sentry_nav/sensor_scan_generation/src/sensor_scan_generation.cpp` | 门控后移（TF/里程计先发布），日志改 `THROTTLE(5000)` |
| `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:239-241` | `max_hessian_cond: 1.0e6` → `1000000.0` |
| `issues/005`、`issues/006`、`issues/README.md` | 文档 |

---

## 验证方法

```bash
# F1：确认 /cmd_vel 无订阅者（真机 bringup 运行中）
ros2 topic info /cmd_vel -v
#   期望修改后 Publisher=1, Subscription=1(/cmd_vel_base_real_yaw 或 remap 后)

# F2：确认 sensor_scan 是否有订阅者
ros2 topic info /sensor_scan -v

# F4：确认裁判话题发布者数量
ros2 topic info /referee/ground_robot_position -v
ros2 topic info /referee/all_robot_hp -v
ros2 topic info /referee/buff -v

# F5：确认实际执行的树
ros2 param get /pb2025_sentry_behavior_client target_tree
#   并对照启动日志 "Sending goal to execute Behavior Tree: ..."
```

---

## 处理记录

| 日期 | 处理人 | 改动 / 结论 |
|------|--------|-------------|
| 2026-10-20 | — | 完成数据流审计，记录 F1~F6；本文件不包含代码改动，待确认修复范围 |
