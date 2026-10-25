# 009. ✅ MPPI 巡航全程爬行、临近目标固定距离突然加速

- **状态**：✅ 已修（配置已改，**待实车复测**）
- **发现日期**：2026-10-23
- **影响面**：功能失效（导航能走完，但巡航速度极低，赛场上等效于超时失败）
- **适用版本**：Nav2 **Humble**（`nav2_mppi_controller` 1.1.18，`dpkg` 实测 `1.1.18-1jammy`）
- **相关文件**：
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:416-492`（MPPI 参数）
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:561-563`（局部膨胀层）
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:618-619`（全局膨胀层）
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/navigation_launch.py`（`/cmd_vel` 链路）
  - `src/pb2025_sentry_behavior/launch/sentry_tactical_real_test.launch.py:63-65`（**0.3 m/s 硬上限**）

---

## 1. 现象

> "规划路径后机器人移动会在最后末尾一段固定距离快速移动，而在其他时段则慢速移动。"

拆成两个**互相独立**的症状：

| 症状 | 表现 | 成因（见 §4） |
|------|------|----------------|
| A. 末端冲刺 | 距目标约 2~5 m 开始明显加速 | `GoalCritic` 的 `threshold_to_consider` / `cost_weight` |
| B. 其余路段爬行 | 全场匀速慢，与 `vx_max` 调到多少无关 | `ObstaclesCritic` 的 `cost_power: 2` + `collision_margin_distance: 0.30` |

单独修 A 或单独修 B 都只能改善一半，必须一起改。

**关键反证**：曾把 `vx_max` 从 `2.5` 提到 `5.0`（同时把局部地图 8 m 扩到 10 m），速度**没有任何改善** —— 说明瓶颈不是速度上限，而是代价函数把"快"这条轨迹直接判死。

---

## 2. `/cmd_vel` 实际链路（排查前先确认改的是哪一段）

`navigation_launch.py` 的 remap 决定了控制指令要穿过 4 个节点：

```
controller_server ──/cmd_vel_controller──> velocity_smoother
                        (MPPI 原始输出)          │
                                                │ /cmd_vel_nav2_result
                                                ▼
                                        fake_vel_transform
                                                │ /cmd_vel_base_real_yaw
                                                ▼
                                        rm_serial_driver → 底盘
```

（`navigation_launch.py:208`、`231-232`、`259`；`fake_vel_transform` 的输入/输出见 `nav2_params.yaml:154-155`）

所以"慢"可能发生在三段中的任意一段：

```bash
ros2 topic hz   /cmd_vel_controller
ros2 topic echo /cmd_vel_controller   --once   # MPPI 到底要了多快
ros2 topic echo /cmd_vel_nav2_result  --once   # 经过 velocity_smoother 之后
ros2 topic echo /cmd_vel_base_real_yaw --once  # 真正发给底盘
ros2 topic echo /odometry --field twist        # 底盘实际跑多快
```

结论定位：

| 现象 | 责任段 |
|------|--------|
| `/cmd_vel_controller` 本身就很慢 | MPPI（critic 权重失衡，本文主题） |
| `/cmd_vel_controller` 快、`/cmd_vel_nav2_result` 慢 | `velocity_smoother`（`max_velocity` / `max_accel`） |
| 前两者都快、底盘不动 | `fake_vel_transform` 或串口/底盘 |
| 以上都快，但 `odometry` 显示慢 | 底盘功率限制 / 打滑 |

**另外**：`fake_vel_transform` 用 `message_filters::ApproximateTime` 把 `odometry`（10 Hz，雷达帧率）和 `local_plan`（20 Hz）配对后才转发 `cmd_vel`（`fake_vel_transform.cpp:139-146`），因此**下发频率实际被 odom 的 10 Hz 限住**。这是抖动来源，但不构成速度上限，本文不展开。

### 2.1 实际生产启动命令（**不含任何速度门控**）

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  use_composition:=True slam:=False use_robot_state_pub:=True
```

这条路径的参数改写只有 `use_sim_time` / `autostart` / `yaml_filename`，**没有任何速度覆盖**：

| 层级 | 文件 | `param_rewrites` |
|---|---|---|
| `rm_navigation_reality_launch.py:138-146` | 传给 `bringup_launch.py` | `{}`（空） |
| `bringup_launch.py:59` | 传给 `navigation_launch.py` | `use_sim_time`, `yaml_filename` |
| `navigation_launch.py:38` | 传给各 nav2 节点 | `use_sim_time`, `autostart` |

因此 `nav2_params.yaml` 里的 `vx_max` / `velocity_smoother.max_velocity` **就是实际生效值**，
`FollowPath` 与 `velocity_smoother` 的参数会原样下发。

三个容易踩的细节：

1. **`slam:=False` 时 `map` 的默认值是 `map/reality/GX_test_show.yaml`**（`rm_navigation_reality_launch.py:73-77`），
   **不是** `map/RMUC26.yaml`。要跑 RMUC 场地必须显式传 `map:=.../map/RMUC26.yaml`，
   否则全局代价地图来自另一张图（只影响全局规划，不影响本文的局部速度问题）。
2. **`use_composition:=True` 并不包含 `controller_server`**：它仍在 composable 列表之外，
   以独立进程启动（`navigation_launch.py:255-265`，且无条件 `add_action`）。
   容器里装的是 `smoother_server / planner_server / behavior_server / bt_navigator /
   waypoint_follower / velocity_smoother / loam_interface / sensor_scan_generation /
   fake_vel_transform / lifecycle_manager`。
   所以 `local_costmap`（10 m @ 25 Hz）与 MPPI（1200×50）**依旧在同一进程里抢 CPU**，
   开组合模式并不能缓解这一点（对照 issue 008 §1）。
3. `use_robot_state_pub:=True` 由 `rm_navigation_reality_launch.py:148-153` 处理；
   传给 `bringup_launch.py` 时该参数被硬编码为 `"False"`（同文件 `:200`），避免重复发布 TF。

---

## 3. 证据

### 3.1 `ObstaclesCritic` 的代价会被平方（代码验证）

`nav2_mppi_controller/src/critics/obstacles_critic.cpp`（1.1.18）：

```cpp
data.costs += xt::pow(
  (critical_weight_ * raw_cost) +                       // 近距离严惩
  (repulsion_weight_ * repulsive_cost / traj_len),      // 一般排斥
  power_);                                             // ← cost_power: 2 时整体平方
```

而 `PathFollowCritic` / `PathAlignCritic` / `GoalCritic` 的权重都在 `×10⁰` 量级。
一旦 `power_ = 2`，障碍项会被放大到 `×10³ ~ 10⁴`，**其余 critic 全部失去话语权**。

### 3.2 `distanceToObstacle()` 返回的是"离障碍**表面**的距离"（代码验证）

```cpp
float dist_to_obj = (scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor;
if (!cost.using_footprint) {
  dist_to_obj -= min_radius;   // ← 扣除内切半径
}
```

把膨胀层公式 `cost = 252·exp(-scale·(d - 内切半径))` 代回去：

```
dist_to_obj ≈ d - 内切半径   （d = 障碍中心距）
```

数值验证（内切半径 0.24 m，即 footprint 0.48 m 方形的一半）：

| 中心距障碍 d | 原配置代价（radius 0.35 / scale 10） | `dist_to_obj` |
|---|---|---|
| 0.24 m | 252.0 | 0.000 m |
| 0.28 m | 168.9 | 0.040 m |
| 0.35 m（膨胀带边缘） | 83.9 | **0.110 m** |

**注意**：原 `inflation_radius: 0.35` 只比内切半径 0.24 宽 **11 cm**，因此整条膨胀带里 `dist_to_obj ∈ [0, 0.11]`。

### 3.3 `collision_margin_distance: 0.30` 比整条膨胀带还宽

```cpp
if (dist_to_obj < collision_margin_distance_) {        // 0 ≤ dist ≤ 0.11 < 0.30，恒成立
  traj_cost += (collision_margin_distance_ - dist_to_obj);   // 每个点 +0.19 ~ +0.30
}
```

**膨胀带内的每一个轨迹点都触发严惩。** 该参数的本意是"防止贴近碰撞"，实际变成了"禁止进入膨胀带"。

### 3.4 量级对比：障碍代价 vs 路径跟随代价

`critical_weight: 20`、`cost_power: 2`、`traj_len = time_steps = 50`：

| 轨迹落在膨胀带内的点数 | 每点 `traj_cost` | 平方后 `ObstaclesCritic` 代价 |
|---|---|---|
| 5 点 | ≈ 0.2 | **≈ 400** |
| 15 点 | ≈ 0.2 | **≈ 3 600** |
| 30 点 | ≈ 0.2 | **≈ 14 400** |

对照其他 critic 的理论最大值：

| critic | 最大量级 |
|---|---|
| `PathFollowCritic`（`cost_weight 6`） | ≈ 36 |
| `GoalCritic`（`cost_weight 5`） | ≈ 15 |
| `ConstraintCritic`（速度未越界） | 0 |
| `CostCritic`（`cost_weight 2.5`，已按 254 归一化） | < 1 |

**差 2~3 个数量级。** 于是 MPPI 在每一步唯一在做的事是"最小化在膨胀带内的点数"，
即：**减速 → 停住 → 等障碍出带**。RM 场地上障碍几乎无处不在 → 全程爬行；
目标附近的空地没有障碍 → 突然全速。**症状 A 与 B 因此被同一个机制串起来。**

### 3.5 `GoalCritic` 的"阈值即开关"语义（代码验证）

```cpp
void GoalCritic::score(CriticData & data)
{
  if (!enabled_ || !utils::withinPositionGoalTolerance(threshold_to_consider_, ...))
    return;                            // 仅在 距离 < threshold 时才生效
  ...
  data.costs += xt::pow(xt::mean(dists, {1}, immediate) * weight_, power_);
}
```

`withinPositionGoalTolerance` 是 `dist < tolerance → true`（`tools/utils.hpp:233`）。
所以 `threshold_to_consider` 是一个**硬开关**：

- 原配置 `threshold_to_consider: 5.0` + `cost_weight: 20.0` → 最后 **5 m** 突然加入一个权重 20 的"冲向目标"项；
- 同时 `PathFollowCritic` / `PathAlignCritic` 的 `threshold_to_consider: 1.0` → 1 m 内路径跟踪全部关闭。

三个阈值错开，就形成了"末端固定距离冲刺"。

### 3.6 `ax_*` / `ay_*` / `az_*` / `vy_min` / `wz_min` 是无效参数（代码验证）

`/opt/ros/humble/include/nav2_mppi_controller/models/constraints.hpp`：

```cpp
struct ControlConstraints
{
  float vx_max;
  float vx_min;
  float vy;    // 只有 vy_max
  float wz;    // 只有 wz_max
};
```

MPPI 1.1.18 只读取 `vx_max / vx_min / vy_max / wz_max`。
配置里"加速度提高，高速时能快速响应""制动力必须对称"这些注释对应的 `ax_max / ax_min / ...` **完全不生效**，
真正的加减速由 `velocity_smoother.max_accel / max_decel` 决定（当前 `2.5 / -2.5 m/s²`）。

### 3.7 `vx_max: 5.0` 反而是负优化

MPPI 每个周期从 `N(上一次控制量, vx_std)` 采样 1200 条轨迹。

- 局部地图 `10 m × 10 m`，滚动窗口以机器人为中心 → 前方只有 **5 m** 可用；
- 预测时域 `time_steps × model_dt = 50 × 0.05 = 2.5 s`；
- `vx_max = 5.0` → 最快样本要看到 `5.0 × 2.5 = 12.5 m`，**必然越过地图边界**；
- 越界点在 `CostCritic` 里返回 `NO_INFORMATION`，而局部 costmap 顶层 `track_unknown_space` 未设置（默认 `false`），
  `inCollision()` 把 `NO_INFORMATION` 判为碰撞 → `collision_cost: 100000`。

结果：所有"跑得快"的样本都被直接淘汰，**并且白白浪费采样预算**（`vx_std: 0.6` 相对 5 m/s 太小，样本大量聚集在不可行区）。

### 3.8 二级约束：局部地图半径 vs 预测距离

即使把 `vx_max` 调回 2.5：

```
理论速度上限 ≈ 局部地图半径 / 预测时域 = 5 m / 2.5 s = 2.0 m/s
```

所以 `vx_max: 2.5` 在这个 10 m 窗口下实际会被软性截断在 ≈ 2 m/s。
要真正跑满 2.5 m/s，需要把窗口扩到 12 m（CPU 随面积线性增长），或把 `time_steps` 降到 40（时域 2.0 s）。
本条属于"上限"，不是"爬行"的原因，**先解决 §3.1~3.4 再考虑**。

### 3.9 先排除：测试 launch 里的 0.3 m/s 硬上限

```python
# src/pb2025_sentry_behavior/launch/sentry_tactical_real_test.launch.py:63-65
smoother.update(max_velocity=[0.3, 0.3, 0.5], min_velocity=[-0.3, -0.3, -0.5])
controller.update(vx_max=0.3, vx_min=-0.3, vy_max=0.3, wz_max=0.5)
```

这份文件会**运行时覆盖** `nav2_params.yaml`，把速度锁死在 0.3 m/s。
如果用的是它，本文所有改动都不会生效。`sentry_decision_real.launch.py` 没有此覆盖。

**生产实际使用的是 §2.1 的命令**（`rm_navigation_reality_launch.py`），该路径无任何速度覆盖，
所以本节的 0.3 m/s 只影响用测试 launch 的场景，不参与本文的根因。

---

## 4. 根因

| # | 根因 | 机制 |
|---|------|------|
| 1 | `ObstaclesCritic.cost_power: 2` | 障碍代价被平方，量级 10³~10⁴，压过所有其他 critic |
| 2 | `ObstaclesCritic.collision_margin_distance: 0.30` | 该阈值比较"离障碍表面距离"，而膨胀带只有 11 cm → **带内每点都触发严惩** |
| 3 | `inflation_radius: 0.35` + `cost_scaling_factor: 10` | 膨胀带只有 11 cm（内切 0.24 → 0.35），既无梯度，又让根因 2 覆盖全带；`CostCritic` 也因此退化成硬惩罚 |
| 4 | `GoalCritic.threshold_to_consider: 5.0` + `cost_weight: 20` | 最后 5 m 硬开一个权重 20 的目标项 → 末端冲刺（根因 1~3 修好后仍需调，否则落差更明显） |

---

## 5. 方案（已实施）

`src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`

| 位置 | 参数 | 原值 | 新值 | 理由 |
|------|------|------|------|------|
| 471 | `ObstaclesCritic.cost_power` | `2` | **`1`** | 去掉平方，量级回到与路径项可比 |
| 476 | `ObstaclesCritic.collision_margin_distance` | `0.30` | **`0.10`** | 官方默认；只惩罚真正贴障的点 |
| 561 | 局部 `cost_scaling_factor` | `10.0` | **`5.0`** | 梯度平缓，`CostCritic` 才能提供连续排斥 |
| 563 | 局部 `inflation_radius` | `0.35` | **`0.55`** | 给出 31 cm 平滑梯度带（外接半径 0.339 + 21 cm 余量） |
| 619 | 全局 `inflation_radius` | `0.30` | **`0.55`** | 全局路径不再贴障 |
| 466-467 | `GoalCritic.cost_weight` / `threshold_to_consider` | `20.0` / `5.0` | **`5.0` / `2.0`** | 消除末端突变，与路径 critic 的 1.0 m 阈值衔接 |
| 426-430 | `vx_max` / `vy_max` / `vx_min` / `vy_min` | `5.0` / `5.0` / `-5.0` / `-5.0` | **`2.5` / `2.5` / `-2.5` / `-2.5`** | 5 m/s 对 RM 底盘无意义，且采样全部飞出地图 |
| 422-423 | `vx_std` / `vy_std` | `0.6` | **`0.3`** | 取期望巡航速度的 20~30%，采样效率最高 |
| 447 | `temperature` | `0.5` | **`0.3`** | 官方默认 |
| 448 | `gamma` | `0.0` / `0.1` | **`0.015`** | `0` 会让末端加速突兀；`0.1` 过度平滑导致响应迟钝 |

### 改后代价分布（单点，`power=1 / weight=20 / margin=0.10 / radius=0.55`）

| 距障碍表面 | 严惩项 | 排斥项 | 单点合计 |
|---|---|---|---|
| 0.02 m | 1.60 | 0.021 | **1.62** |
| 0.08 m | 0.40 | 0.019 | **0.42** |
| 0.15 m | 0.00 | 0.016 | **0.016** |
| 0.30 m | 0.00 | 0.010 | **0.010** |

只有真正贴上去才重罚；15 cm 以外基本不干预，速度重新由 `PathFollowCritic`（最大 ≈36）决定。

### 未采用的做法及原因

| 做法 | 不采用的原因 |
|---|---|
| 只调大 `vx_max` | 已实测无效（§3.7），瓶颈在代价函数 |
| 直接删掉 `ObstaclesCritic` | 可行（Nav2 官方默认配置只用 `CostCritic`），但需先把膨胀带做宽；本方案保留 critic、只修正权重，改动更小、便于回滚 |
| 把 `inflation_radius` 提到 0.65 | 会挤占 0.9 m 窄道的可通行空间；0.55 已足够提供梯度 |
| 调 `ax_max` / `ax_min` | MPPI 不读这些参数（§3.6） |

---

## 6. 验证方法

### 6.1 改参数后先做静态检查

```bash
# YAML 能被解析 & 参数确实生效（改完先跑这条）
python3 -c "import yaml;d=yaml.safe_load(open('src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml'));print(d['controller_server']['ros__parameters']['FollowPath']['ObstaclesCritic'])"

# 节点起来后确认运行时值（防止被 launch 覆盖）
# 生产命令：rm_navigation_reality_launch.py use_composition:=True slam:=False use_robot_state_pub:=True
ros2 param get /controller_server FollowPath.ObstaclesCritic.cost_power            # 期望 1
ros2 param get /controller_server FollowPath.ObstaclesCritic.collision_margin_distance  # 期望 0.10
ros2 param get /controller_server FollowPath.vx_max                               # 期望 2.5（非 0.3 / 非 5.0）
ros2 param get /controller_server FollowPath.gamma                                # 期望 0.015
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius       # 期望 0.55
ros2 param get /local_costmap/local_costmap inflation_layer.cost_scaling_factor    # 期望 5.0
ros2 param get /velocity_smoother max_velocity                                    # 期望 [2.5, 2.5, 3.0]，不是 [0.3, 0.3, 0.5]

# 控制周期是否跑满（10 m 窗口 @25 Hz + MPPI 1200×50 同进程，CPU 容易吃紧）
ros2 topic hz /cmd_vel_controller      # 期望 ≈20 Hz；若只有 7~10 Hz 说明 controller_server 抢不到 CPU
# 日志里搜：Control loop missed its desired rate
```

### 6.2 跑起来后看速度链

```bash
ros2 topic hz   /cmd_vel_controller
ros2 topic echo /cmd_vel_controller   --once
ros2 topic echo /cmd_vel_base_real_yaw --once
ros2 topic echo /odometry --field twist
```

期望：`/cmd_vel_controller.linear.x` 在开阔区稳定在 **1.5~2.0 m/s**（受 §3.8 的 2 m/s 软上限约束），
经过障碍附近允许短暂降到 0.5~1.0 m/s，**但离开后应立刻回升**，而不是全程一个低值。

### 6.3 定位残余问题

```bash
# 1) critic 是否还在压速度？看各 critic 的代价贡献
ros2 param set /controller_server FollowPath.publish_critics_stats true
#    用 PlotJuggler 订阅 /controller_server/critics_stats，看哪个 critics[i].costs_sum 占主导

# 2) 局部地图里是不是真的有障碍？
ros2 topic echo /local_costmap/costmap --once | head -20
#    或 RViz 打开 local_costmap/costmap，颜色发蓝/紫说明膨胀带被灌满

# 3) 是否被测试 launch 覆盖
ros2 param get /controller_server FollowPath.vx_max    # 若为 0.3，说明用了 sentry_tactical_real_test.launch.py
```

### 6.4 回归检查

- [ ] 开阔区巡航 ≥ 1.5 m/s
- [ ] 距目标 2 m 内不再出现速度突变
- [ ] 单障碍绕行时不再出现"停住等待"，能绕行通过
- [ ] 0.9 m 窄道仍可通过（膨胀 0.55 后需实测）
- [ ] `progress_checker` 不再触发（`required_movement_radius 0.5 / 10 s`）

---

## 7. 相关文件

| 文件 | 作用 |
|------|------|
| `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml` | 全部改动所在 |
| `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/navigation_launch.py:208,231-232,259` | `/cmd_vel` remap 链路 |
| `src/pb2025_sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp:139-146` | `ApproximateTime` 同步后才转发 `cmd_vel`（10 Hz 下发） |
| `src/pb2025_sentry_behavior/launch/sentry_tactical_real_test.launch.py:63-65` | **0.3 m/s 硬上限（排查项）** |
| `/opt/ros/humble/include/nav2_mppi_controller/models/constraints.hpp` | `vx_max/vx_min/vy/wz` 四个字段，证明 `ax_*` 等无效 |
| `/opt/ros/humble/include/nav2_mppi_controller/tools/utils.hpp:233` | `withinPositionGoalTolerance` 语义 |
| 上游 `nav2_mppi_controller` 1.1.18 `src/critics/obstacles_critic.cpp` | `distanceToObstacle()`、`score()` 的平方项 |
| 上游 `nav2_mppi_controller` 1.1.18 `src/critics/goal_critic.cpp` | `threshold_to_consider` 的硬开关语义 |

---

## 8. 处理记录

| 日期 | 处理人 | 改动 / 结论 |
|------|--------|-------------|
| 2026-10-23 | — | 定位根因为 `ObstaclesCritic` 二次代价 + `collision_margin_distance` 覆盖整条 11 cm 膨胀带；已按 §5 修改配置（10 个参数），`python3 yaml.safe_load` 验证通过；**尚未实车复测** |
|  |  |  |

### 待办

- [ ] 实车复测 §6.2 / §6.4
- [ ] 若开阔区仍上不去 1.5 m/s，按 §3.8 二选一：窗口 10 m → 12 m，或 `time_steps` 50 → 40
- [ ] 若窄道通过受影响，将 `inflation_radius` 在 0.45~0.55 之间回调
