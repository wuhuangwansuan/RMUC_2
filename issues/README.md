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

## 索引

| # | 状态 | 主题 | 影响面 | 文件 |
|---|------|------|--------|------|
| 001 | 🔴 | 双 MID360 时间戳基准跳变 → FAST_LIO 出不了里程计 | 致命：建图/定位/SLAM/导航全链路起不来 | [001-dual-lidar-timestamp-base-jump.md](./001-dual-lidar-timestamp-base-jump.md) |
| 002 | 🔴 | `origin` 地图目录名多一个前导空格 | slam_toolbox 读 origin 图失败（续建图失效） | [002-origin-dir-leading-space.md](./002-origin-dir-leading-space.md) |
| 003 | 🟡 | `mapping_mission_manager` 逻辑/时序疑点 | 建图引导行为与文案不一致、回调阻塞、超时后可能重发导航 | [003-mapping-mission-manager-suspects.md](./003-mapping-mission-manager-suspects.md) |
| 004 | 🟡 | 驱动 `is_timestamp_sync_` 全局共享（两雷达互相污染） | 送帧节拍在两套时钟间横跳（001 的子问题） | [004-livox-timestamp-sync-flag.md](./004-livox-timestamp-sync-flag.md) |

## 约定

- 文件命名：`NNN-短横线小写英文描述.md`，序号递增、不复用。
- 每份内容保持统一结构：**元信息 → 现象 → 证据 → 根因/疑点 → 方案 → 验证 → 相关文件**。
- 新增问题：复制 [TEMPLATE.md](./TEMPLATE.md)，填好后登记到上面的「索引」表。
- 修好后把状态改为 ✅ 并在文件末尾「处理记录」写明改动与验证结果，**不删除文件**。

## 变更记录

| 日期 | 变更 |
|------|------|
| 2026-10-04 | 建立 `issues/` 目录；由根目录 `issue.md` 拆分录入 001 / 002 / 003 / 004 |
