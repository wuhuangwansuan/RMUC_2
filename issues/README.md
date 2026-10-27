# 问题追踪（Issue Log）

> 用途：记录**暂不立即 debug**、但已确认存在或有疑点的问题及其分析与解决方案。
> 本目录每个问题一个文件，本 README 作为索引。

## 状态标记

| 标记 | 含义 |
|------|------|
| 🔴 | 待修（已确认存在） |
| 🟡 | 观察 / 存疑（尚未确认或暂不处理） |
| ✅ | 已修（保留记录） |
| ⚪ | 已排除 |
| 📘 | 参考文档（非缺陷记录） |

## 索引

| # | 状态 | 主题 | 影响面 | 文件 |
|---|------|------|--------|------|
| 001 | 🔴 | 双 MID360 时间戳基准跳变 → FAST_LIO 出不了里程计 | 致命：建图/定位/SLAM/导航全链路起不来 | [001-dual-lidar-timestamp-base-jump.md](./001-dual-lidar-timestamp-base-jump.md) |
| 002 | 🔴 | `origin` 地图目录名多一个前导空格 | slam_toolbox 读 origin 图失败（续建图失效） | [002-origin-dir-leading-space.md](./002-origin-dir-leading-space.md) |
| 003 | 🟡 | `mapping_mission_manager` 逻辑/时序疑点 | 建图引导行为与文案不一致、回调阻塞、超时后可能重发导航 | [003-mapping-mission-manager-suspects.md](./003-mapping-mission-manager-suspects.md) |
| 004 | 🟡 | 驱动 `is_timestamp_sync_` 全局共享（两雷达互相污染） | 送帧节拍在两套时钟间横跳（001 的子问题） | [004-livox-timestamp-sync-flag.md](./004-livox-timestamp-sync-flag.md) |
| 005 | 🔴 | `small_gicp_relocalization` 注册回调阻塞同节点 TF 发布 | 致命：`map→odom` 冻结 → nav2 目标变换失败、`follow_path` abort | [005-small-gicp-registration-blocks-tf-publish.md](./005-small-gicp-registration-blocks-tf-publish.md) |
| 006 | ✅ | 粗配准在 0.1 m 全图上做 FPFH，单次注册 65 s | 致命：重定位耗时分钟级（已改为两级体素，离线实测 0.23 s） | [006-coarse-registration-full-map-fpfh.md](./006-coarse-registration-full-map-fpfh.md) |
| 007 | 🔴 | 导航 ⇄ 决策数据流审计（`/cmd_vel` 断链等 F1~F6） | 功能失效：未开赛停车无效；建图门控未覆盖实际链路；战术决策降级 | [007-nav-decision-dataflow-audit.md](./007-nav-decision-dataflow-audit.md) |
| 008 | 📘 | `local_costmap` 参数详解与调参指南 | 调参效率（含 3 处注释错误与竖直窗口 0.8 m 陷阱） | [008-local-costmap-params-tuning.md](./008-local-costmap-params-tuning.md) |
| 009 | ✅ | MPPI 巡航全程爬行、临近目标固定距离突然加速 | 功能失效：障碍代价压过所有 critic → 只减速不前进（已改配置，待实车复测） | [009-mppi-cruise-slow-goal-sprint.md](./009-mppi-cruise-slow-goal-sprint.md) |
| 010 | 🟡 | `local_costmap` / `global_costmap` 参数与数学原理梳理 | 导航安全边界与调参依据（与 008 互补：008 聚焦 local 调参，本篇含 global 与源码级推导） | [010-costmap-local-global-params.md](./010-costmap-local-global-params.md) |

## 约定

- 文件命名：`NNN-短横线小写英文描述.md`，序号递增、不复用。
- 每份内容保持统一结构：**元信息 → 现象 → 证据 → 根因/疑点 → 方案 → 验证 → 相关文件**。
- 新增问题：复制 [TEMPLATE.md](./TEMPLATE.md)，填好后登记到上面的「索引」表。
- 修好后把状态改为 ✅ 并在文件末尾「处理记录」写明改动与验证结果，**不删除文件**。

## 变更记录

| 日期 | 变更 |
|------|------|
| 2026-10-04 | 建立 `issues/` 目录；由根目录 `issue.md` 拆分录入 001 / 002 / 003 / 004 |
| 2026-10-20 | 新增 005：`small_gicp_relocalization` 注册阻塞 TF 发布（含最小复现程序） |
| 2026-10-20 | 新增 006：粗配准全图 FPFH 导致单次注册 65 s；记录已实施的两级体素改动与离线实测 |
| 2026-10-20 | 新增 007：导航⇄决策数据流审计（F1~F6，含 `/cmd_vel` 断链与建图门控未覆盖） |
| 2026-10-21 | 新增 008：`local_costmap` 参数详解与调参指南（源码级核对，含注释错误与体素竖直窗口陷阱） |
| 2026-10-23 | 新增 009：MPPI 巡航爬行 / 末端冲刺（`ObstaclesCritic` 二次代价与 `collision_margin_distance` 覆盖整条膨胀带） |
| 2026-10-27 | 新增 010：`local_costmap` / `global_costmap` 参数与数学原理梳理（自旧分支 `auto_reloc_checker/_10_9` 的 105 合入，重编号为 010） |
