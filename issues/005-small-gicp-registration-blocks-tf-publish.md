# 005. 🔴 `small_gicp_relocalization` 注册回调阻塞同一节点内的 TF 发布（`map→odom` 冻结）

- **状态**：🔴 待修
- **发现日期**：2026-10-20
- **影响面**：致命：`map→odom` 停止更新 → nav2 无法把目标位姿转到 costmap 系 → `follow_path` 直接 abort
- **相关文件**：
  - `src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp:207`
  - `src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp:211`
  - `src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp:198`
  - `src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp:202`

---

## 现象

启动 `rm_navigation_reality_launch.py use_composition:=True` 后，nav2 各节点均为 active，
行为树下发首个导航目标时立刻失败：

```
[controller_server] [ERROR] Exception in transformPose: Lookup would require extrapolation into the future.
  Requested time 1792497029.312055 but the latest data is at time 1792497025.100286,
  when looking up transform from frame [map] to frame [odom]
[controller_server] [ERROR] Unable to transform goal pose into costmap frame
[controller_server] [WARN]  [follow_path] [ActionServer] Aborting handle.
```

`global_costmap` 同时持续刷：

```
[global_costmap.global_costmap]: Message Filter dropping message: frame 'odom' at time 1792497025.300
  for reason 'the timestamp on the message is earlier than all the data in the transform cache'
```

关键现象：**`map→odom` 的时间戳卡死在 `1792497025.100` 不再前进**，而日志还在继续输出到 `1792497030.044`。

---

## 证据

### 证据 1：TF 发布在注册开始的同一瞬间断掉

日志中 `publishTransform()` 的心跳打印（`||x/||y/||z`，20 Hz）**总共只有 9 条**：

```
[1792497024.646] [small_gicp_relocalization]: ||x:0.000000
[1792497024.696] ...
[1792497024.746] ...
[1792497024.796] ...
[1792497024.846] ...
[1792497024.896] ...
[1792497024.946] ...
[1792497024.996] ...
[1792497025.046] [small_gicp_relocalization]: ||x:0.000000   ← 最后一次
[1792497025.096] [small_gicp_relocalization]: ////Performing registration with 26119 accumulated points ////
                ────────── 之后 4 秒，small_gicp 一行输出都没有 ──────────
[1792497025.173] [lifecycle_manager_navigation]: Server controller_server connected with bond.
[1792497025.276] [lifecycle_manager_navigation]: Activating planner_server
...
[1792497030.044] [controller_server]: [follow_path] [ActionServer] Aborting handle.
```

时间间隔正好 50 ms 一条（20 Hz），在 `Performing registration` 前 50 ms 处截断。
同一容器内其它组件（`lifecycle_manager`、`global_costmap`、`planner_server`）在此期间照常工作。

### 证据 2：两个 timer 都没有指定 callback group

`src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp:207`

```cpp
register_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(500),  // 2 Hz
  std::bind(&SmallGicpRelocalizationNode::performRegistration, this));   // ← 无 group 参数

transform_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(50),   // 20 Hz
  std::bind(&SmallGicpRelocalizationNode::publishTransform, this));      // ← 无 group 参数
```

整个包内 `grep -rn "callback_group\|CallbackGroup"` **无任何匹配**；两个订阅
（`pcd_sub_`、`initial_pose_sub_`）同样未指定 group。

### 证据 3：不传 group 会落到节点的默认互斥组

`/opt/ros/humble/include/rclcpp/rclcpp/node.hpp:238`

```cpp
create_wall_timer(
  std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr);   // ← 默认 nullptr
```

`group == nullptr` 时使用 `node_base->get_default_callback_group()`。
用最小程序把它打印出来：

```
default callback group type = MutuallyExclusive
```

### 证据 4：最小复现实测（rclcpp）

复现程序：一个 500 ms 定时器内 `sleep 3s`（模拟 `performRegistration`），
一个 50 ms 定时器计数（模拟 `publishTransform`）。

| 场景 | 执行器 | 3 s 阻塞期间 50 ms 定时器 tick |
|---|---|---|
| **默认组（当前代码）** | SingleThreaded | **9 → 9 → 9（完全停摆）** |
| 默认组 | **MultiThreaded** | **9 → 9 → 9（仍然停摆）** |
| 拆成两个 MutuallyExclusive 组 | SingleThreaded | **9 → 9 → 9（仍然停摆）** |
| 拆成两个 MutuallyExclusive 组 | **MultiThreaded** | **20 → 39 → 60（正常）** |

当前代码的实测输出：

```
default callback group type = MutuallyExclusive
mode=default executor=SingleThreadedExecutor(1 thread)
  [heavy ] start @ 0.50s
  t=1s  50ms-timer ticks=9  (ideal=20)
  t=2s  50ms-timer ticks=9  (ideal=40)
  t=3s  50ms-timer ticks=9  (ideal=60)
  [heavy ] end   @ 3.50s
```

结论：**只拆 callback group 而不换多线程执行器无效**；
即使换 `MultiThreadedExecutor`，只要仍在默认互斥组里也无效。

### 证据 5：容器给每个组件一个单线程执行器

```
$ nm -DC /opt/ros/humble/lib/rclcpp_components/component_container_isolated | grep -i executor
_Sp_counted_ptr_inplace<rclcpp::executors::SingleThreadedExecutor, ...>
std::thread::_M_start_thread(...)
```

`component_container_isolated` 为每个组件单独创建 `SingleThreadedExecutor` 并在独立线程中 spin。
因此：
- 其它组件（`global_costmap` 等）不受影响 —— 它们在各自线程；
- `small_gicp_relocalization` 内部的 2 个订阅 + 2 个定时器**全部排在同一条线程上串行**。

### 证据 6：单次注册耗时实测

用真实先验地图 `GX_test_show.pcd`（1,098,176 点）+ 合成扫描单独运行该节点：

```
1792497742.186  ////Performing registration with 42988 accumulated points ////
1792497742.136  ||x:0.000000                     ← 最后一次 TF 发布
1792497757.338  [ROUGH] RANSAC inliers: 21 / 15252     （FPFH+RANSAC 耗时 15.2 s）
1792497807.596  [NIS-1] eps_res=37379.4 ...
1792497807.598  Relocalization rejected (converged=0, accepted=1)
1792497807.686  ////Performing registration with 236434 accumulated points ////  ← 立刻开始下一轮
```

**单次 `performRegistration()` 约 65 秒**，期间 `map→odom` 完全不发布。
注意第二轮点数从 21,494 涨到 236,434：注册阻塞期间订阅回调同样被堵，
DDS 队列（depth=10）中的点云在回调恢复后一次性灌入 `accumulated_cloud_`，
形成“越卡越慢”的正反馈。

---

## 根因

1. `performRegistration()`（500 ms 定时器）是一个**同步阻塞**的长任务（实测分钟级）。
2. 它与 `publishTransform()`（50 ms 定时器）、`registeredPcdCallback()` 同属节点默认的
   `MutuallyExclusiveCallbackGroup`，且宿主容器 `component_container_isolated` 使用
   `SingleThreadedExecutor`。
3. 于是注册一旦开始，`publishTransform` 与点云/初始位姿订阅全部无法执行：
   - `map→odom` 冻结在注册开始前的时间戳；
   - nav2 查询到过期变换 → `extrapolation into the future` → `follow_path` abort；
   - `global_costmap` 因 `odom` 系点云时间戳早于 TF 缓存而丢帧；
   - 点云积压 → 下一轮注册点数暴涨 → 阻塞更久（正反馈）。

> 说明：工作区中已加入 P2-2 的 `coarse_leaf_size` 降采样（粗配准降到约 790 点，详见
> [006](./006-coarse-registration-full-map-fpfh.md)），
> 可把单次耗时从分钟级降到亚秒/秒级，**但不会消除本问题**：
> 只要回调仍在 executor 线程内同步执行，TF 在阻塞期间就仍然停发；
> 而 nav2 以 20 Hz 消费 TF，哪怕 1~2 s 的停顿也足以触发同样的 extrapolation 错误。
> 因此本问题需要与配准耗时优化**分别**修复。

---

## 方案

### 方案 A（推荐）：把重计算移出 executor 线程

- `register_timer_` 回调只做“触发 + 防重入检查”，把 `performRegistration()` 投递到工作线程；
- 增加 `std::atomic<bool> registration_busy_{false}`，或用 `std::future` 判活；
- 工作线程内完成配准后只更新 `result_t_ / previous_result_t_ / fine_trust_`（加锁），
  `publishTransform()` 继续由 50 ms 定时器在 executor 线程发布。

优点：与容器类型无关（`component_container` / `component_container_mt` 都能正常工作），
订阅也不再被阻塞，不会出现点云积压。

### 方案 B：改用 `component_container_mt` + 拆分 callback group

- 给 `register_timer_`、`transform_timer_` 各分配独立的 `ReentrantCallbackGroup`；
- 启动参数改为 `container_name:=nav2_container_mt` 或使用 `component_container_mt`。

缺点：改变整个 nav2 栈的执行模型（其它节点也会变成多线程并发），
需要重新评估现有节点（如 MPPI、costmap）的线程安全性；风险高于方案 A。

### 方案 C：单独进程运行（`use_composition:=False`）

无效。`rclcpp::spin(node)` 仍是 `SingleThreadedExecutor`，回调依旧串行。

### 附带建议

- 注册未成功（`converged == false` 或尚未收到首帧扫描）时**不要发布单位阵 `map→odom`**，
  否则下游会认为机器人在 `(0,0)`；
- `publishTransform()` 中 `||x/||y/||z` 三条 20 Hz INFO 日志建议降级为 DEBUG，
  避免刷屏影响日志可读性（本次日志 9 条心跳全被淹没）。

---

## 验证方法

```bash
# 1. 断开裁判系统与真实传感器也能复现：用最小 C++ 程序验证 callback group 行为
#    （见附录）
./cbg_demo default      # 期望 50 ms 定时器停摆
./cbg_demo groups_mt    # 期望 50 ms 定时器正常

# 2. 实机验证：启动后观察 map->odom 时间戳是否连续
ros2 run tf2_ros tf2_echo map odom
#    应持续输出，且与 ros2 topic echo /registered_scan 的时间戳同步前进

# 3. 触发导航，确认不再出现
#    "Lookup would require extrapolation into the future ... frame [map] to frame [odom]"
```

判定标准：注册运行期间 `tf2_echo map odom` 没有超过 0.2 s 的停顿。

---

## 相关文件

- `src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp`
  - `:207` `register_timer_`（500 ms，`performRegistration`）
  - `:211` `transform_timer_`（50 ms，`publishTransform`）
  - `:198` `pcd_sub_`（`registered_scan` 订阅）
  - `:202` `initial_pose_sub_`（`initialpose` 订阅）
- `src/pb2025_sentry_nav/small_gicp_relocalization/include/small_gicp_relocalization/small_gicp_relocalization.hpp`
  - `:103` `bool fine_trust_{false};`
- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`
  - `:199` `input_cloud_topic: "registered_scan"`
  - `:241` `max_hessian_cond: 1000000.0`（不可写成 `1.0e6`，见下）
- `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/localization_launch.py`
  - `:214-218` 以可组合节点方式加载该组件

---

## 附录：最小复现程序

```cpp
// 编译：cmake + find_package(rclcpp)，运行：./cbg_demo [default|default_mt|groups_st|groups_mt]
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "default";
  auto node = std::make_shared<rclcpp::Node>("cbg_demo");

  auto default_group = node->get_node_base_interface()->get_default_callback_group();
  std::printf("default callback group type = %s\n",
    default_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive ?
    "MutuallyExclusive" : "Reentrant");

  rclcpp::CallbackGroup::SharedPtr gA, gB;
  if (mode != "default" && mode != "default_mt") {
    gA = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gB = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  const auto start = std::chrono::steady_clock::now();
  std::atomic<int> ticks{0};

  auto heavy = node->create_wall_timer(500ms, [&]() {
      std::printf("  [heavy ] start @ %.2fs\n",
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
      std::fflush(stdout);
      std::this_thread::sleep_for(3s);
      std::printf("  [heavy ] end   @ %.2fs\n",
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
      std::fflush(stdout);
    }, gA);

  auto light = node->create_wall_timer(50ms, [&]() {ticks++;}, gB);

  rclcpp::Executor::SharedPtr exec =
    (mode == "groups_mt" || mode == "default_mt")
    ? std::static_pointer_cast<rclcpp::Executor>(
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>())
    : std::static_pointer_cast<rclcpp::Executor>(
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>());
  exec->add_node(node);
  std::thread spinner([&]() {exec->spin();});

  for (int s = 1; s <= 8; ++s) {
    std::this_thread::sleep_for(1s);
    std::printf("  t=%ds  50ms-timer ticks=%d  (ideal=%d)\n", s, ticks.load(), s * 20);
    std::fflush(stdout);
  }
  exec->cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
```

实测结果见「证据 4」。

---

## 处理记录

| 日期 | 处理人 | 改动 / 结论 |
|------|--------|-------------|
| 2026-10-20 | — | 定位到此问题；最小复现程序确认默认互斥组 + 单线程执行器会饿死 TF 定时器。待实施方案 A |
