# 002. 🔴 `origin` 地图目录名多一个前导空格

- **状态**：🔴 待修
- **发现日期**：2026-10-04
- **影响面**：功能失效 —— slam_toolbox 读 origin 图失败（续建图/定位辅助失效），并掩盖真实故障信号
- **相关文件**：
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/map/ origin/`（真实目录名带前导空格）
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/slam_launch.py:92`
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/bringup_launch.py:154`
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:221`

## 现象

```
[sync_slam_toolbox_node-12] [ERROR] serialization::Read: Failed to open requested file:
  /home/fwy/RMUC_2/install/pb2025_nav_bringup/share/pb2025_nav_bringup/map/origin/origin.
[sync_slam_toolbox_node-12] [ERROR] DeserializePoseGraph: Failed to read file:
  /home/fwy/RMUC_2/install/pb2025_nav_bringup/share/pb2025_nav_bringup/map/origin/origin.
```

## 证据

launch 里组装的是 `map/origin/origin`：

- `slam_launch.py:92`：`os.path.join(bringup_dir, "map", "origin", "origin")`
- `bringup_launch.py:154`：同上
- 参数注入：`slam_launch.py:143`（`{"map_file_name": origin_map_file_name}`）、`nav2_params.yaml:221`（`map_file_name: origin_map_file_name`）

但仓库里实际目录名是 **` origin`（开头带一个空格）**：

```
$ ls -b src/pb2025_sentry_nav/pb2025_nav_bringup/map
\ origin        # ← 目录名以空格开头
  reality

$ git ls-files src/pb2025_sentry_nav/pb2025_nav_bringup/map
src/pb2025_sentry_nav/pb2025_nav_bringup/map/ origin/origin.data
src/pb2025_sentry_nav/pb2025_nav_bringup/map/ origin/origin.posegraph
```

## 根因

目录名与代码里拼出的路径不一致（多一个前导空格），`map/origin/origin` 不存在，slam_toolbox 反序列化 posegraph 失败。

## 方案

```bash
git mv "src/pb2025_sentry_nav/pb2025_nav_bringup/map/ origin" \
       "src/pb2025_sentry_nav/pb2025_nav_bringup/map/origin"
# 重新 colcon build，或清理 install 下的旧拷贝
```

验证：`ls -b .../map` 下应出现无空格的 `origin`，slam_toolbox 不再报 `Failed to read file .../map/origin/origin`。

> 待确认：`map_file_name` 指向 `origin` 是设计意图还是残留占位（`nav2_params.yaml:221` 注释写着"现在还没有生成 .posegraph 格式的地图"）。若暂不需要续建图，也可把该参数清空以消除报错。

## 处理记录

| 日期 | 处理人 | 改动 / 结论 |
|------|--------|-------------|
| 2026-10-04 | — | 录入；尚未修改 |
