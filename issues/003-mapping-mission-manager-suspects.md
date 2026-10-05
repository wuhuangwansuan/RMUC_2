# 003. 🟡 `mapping_mission_manager` 逻辑/时序疑点

- **状态**：🟡 观察 / 存疑（未在运行中直接触发或未验证）
- **发现日期**：2026-10-04
- **影响面**：建图引导阶段行为与文案不一致、回调阻塞、超时后可能重发导航
- **相关文件**：
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/src/mapping_mission_manager.cpp`

## 现象

静态阅读发现多处不一致/可疑点，尚未逐一复现。

## 疑点清单

1. **导航点数量与文案不一致**
   - `loadGoal("const_origin_point")`（`:72`）把原点也塞进 `goals_`，加上 `goal1/2/3` 共 **4 个点**。
   - 但 `:201` 硬编码 `"/3"`，`:186` 文案写"导航点已全部下发"，与"3 个点"描述不符。
   - 实际会依次导航 4 个点，日志会显示 `1/3 … 4/3`。

2. **超时时间文案写死 20 秒**
   - `:164`、`:186`、`:241` 均写"20 秒"。
   - 但 `bootstrap_timeout_sec` 默认 `20.0`（`:54`），而实际 `mapping_bootstrap.yaml` 配的是 `120.0`。文案与实际不符。

3. **回调内阻塞调用**
   - `:175` `nav_client_->wait_for_action_server(2s)` 为阻塞等待。
   - `:139` `lifecycle_client_->get_state(std::chrono::seconds(1))` 为阻塞调用，且它在 `onGameStatus` 里**每次收到 GameStatus 都会执行**（`bt_navigator` 未激活期间每帧一次，最长阻塞 1 s）。单线程 executor 下可能拖慢其它回调（TF、timer）。

4. **超时取消后可能继续发下一个导航点**
   - `onBootstrapTimeout`（`:236`）先 `async_cancel_all_goals()`（`:242`）再 `saveMaps()`。
   - 但 `bootstrap_finished_` 要到栅格图保存成功才置 true。这段窗口内 `onNavResult`（`:212`）收到 `CANCELED` 属于非 SUCCEEDED 分支，会 `++goal_index_` 并 `sendNextGoal()`（`:222-227`）——即"取消后可能重新下发导航"，与"取消引导导航"的意图矛盾。**待确认**。

5. **重试定时器自重置的生命周期风险**
   - `:179` 在 `sendNextGoal` 内创建 `retry_goal_timer_`；`:209` 在 `retrySendGoal` 里 `retry_goal_timer_.reset()` 销毁当前定时器后再调用 `sendNextGoal()`（可能再创建）。
   - 在定时器自身回调里 reset 自己所属的 timer 存在生命周期风险，需确认 executor 行为。

6. **`const_origin_point` 语义不清**
   - `:60` 默认定义、`:72` 当作导航点加入 `goals_`，注释未说明它和原点/起始位的区别，容易误用。

## 根因 / 疑点

以上均为静态阅读发现，属于"设计/实现与文档/日志不一致，或存在时序风险"的类别，未确认是否在实车触发。

## 方案

- 暂不修改。待实车复现或明确需求后，再逐条决策：
  - 统一导航点数量与文案（或明确 `const_origin_point` 是否算作一个真正的导航点）；
  - 用参数值动态生成"XX 秒"文案；
  - 将阻塞调用改为异步/降频（例如 `onGameStatus` 用定时器轮询 bt_navigator 状态，而非每帧阻塞查询）；
  - 超时取消后置一个 `cancelling_` 标志，屏蔽随后的 `onNavResult` 重发；
  - 重试定时器改为 `onGameStatus` 里常驻定时器或使用 `one_shot` 语义，避免自 reset。

## 验证方法

复现建图引导全流程（GameStatus→progress=4），观察日志中：
- 是否存在 `4/3` 的导航点编号；
- 超时 120 s 时文案是否仍写 20 秒；
- 超时取消后是否仍有新的 `navigate_to_pose` goal 下发。

## 处理记录

| 日期 | 处理人 | 改动 / 结论 |
|------|--------|-------------|
| 2026-10-04 | — | 录入；未修改 |
