# 006. ✅ 粗配准在 0.1 m 全图上做 FPFH，单次注册耗时 65 s（已改，待实机验证）

- **状态**：✅ 已修（代码已改 + 离线实测；待实机复测）
- **发现日期**：2026-10-20
- **影响面**：致命：单次重定位耗时分钟级，期间 `map→odom` 停发，导航直接失败
- **相关文件**：
  - `src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp:183`
  - `src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp:405`
  - `src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp:505`
  - `src/pb2025_sentry_nav/small_gicp_relocalization/include/small_gicp_relocalization/small_gicp_relocalization.hpp:79`
  - `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:181`
- **关联问题**：[005](./005-small-gicp-registration-blocks-tf-publish.md)（注册回调同步阻塞 TF 发布）

---

## 现象

`small_gicp_relocalization` 每 500 ms 触发一次 `performRegistration()`，一旦进入粗配准
（`fine_trust_ == false`），节点在数十秒内没有任何输出，`map→odom` 完全停止更新。

用真实先验地图 `GX_test_show.pcd`（1,098,176 点）+ 合成扫描单独运行该节点，实测时间线：

```
1792497742.186  ////Performing registration with 42988 accumulated points ////
1792497742.136  ||x:0.000000                  ← 最后一次 TF 发布
1792497757.338  [ROUGH] RANSAC inliers: 21 / 15252     （FPFH + RANSAC 耗时 15.2 s）
1792497807.596  [NIS-1] eps_res=37379.4  ...
1792497807.598  [NIS] relocalization ACCEPTED
1792497807.598  Relocalization rejected (converged=0, accepted=1)
1792497807.686  ////Performing registration with 236434 accumulated points ////  ← 下一轮
```

- **单次 `performRegistration()` = 65.4 s**；
- 期间点云订阅也被阻塞，恢复后 DDS 队列一次性灌入，累积点数 21,494 → 236,434；
- `converged=0`（GICP 20 次迭代内不收敛），`result_t_` 始终为单位阵，
  发布的 `map→odom` 一直是 `x=y=z=0`。

---

## 证据

### 证据 1：`rough_target` 只是“换类型”，没有降采样

修改前（`git show HEAD:.../small_gicp_relocalization.cpp`）：

```cpp
if(!fine_trust_)
{
  //抛开协方差置信度表示，仅将降采样后的点云复制给rough_target
  rough_target->resize(target_->size());
  for (size_t i = 0; i < target_->size(); ++i) {
      rough_target->points[i].x = target_->points[i].x;
      rough_target->points[i].y = target_->points[i].y;
      rough_target->points[i].z = target_->points[i].z;
  }
  rough_target->width  = target_->width;
  rough_target->height = target_->height;
}
```

`target_` 已经按 `global_leaf_size: 0.1` 体素化，这段只是把 `PointCovariance` 逐点拷成
`PointXYZ`（为了喂给 `pcl::FPFHEstimation`），**分辨率仍是 0.1 m**。
且它在**构造函数里只执行一次**，与“置信度低”无关（`fine_trust_{false}` 是初值）。

### 证据 2：先验地图在不同 leaf 下的点数（实测）

对 `GX_test_show.pcd` 做体素化统计：

| leaf | 体素数 | 相对 0.1 m |
|---:|---:|---:|
| **0.1（修改前实际值）** | **135,596** | 1.0× |
| 0.3 | 17,343 | 7.8× |
| 0.5 | 6,308 | 21.5× |
| **1.5（YAML 里的粗配准 leaf）** | **666** | **203.6×** |

`determineCorrespondences` 是暴力 O(N_s × N_t)：约 1.5 万 × 13.5 万 ≈ 20 亿次距离计算，
这就是 15.2 s 的来源。

### 证据 3：YAML 里配了参数，但代码从未 declare

修改前：

```bash
$ grep -c "coarse_leaf_size\|fine_leaf_size\|coarse_max_iterations\|fine_max_iterations" \
        src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp
0
```

`nav2_params.yaml` 里写了 `coarse_leaf_size: 1.5` / `fine_leaf_size: 0.3` 等，
但节点从未 `declare_parameter`，rclcpp 会静默忽略 —— 配了等于没配。

### 证据 4：法线半径硬编码 0.1 m

修改前 `rough_match()` 中：

```cpp
normal_.setRadiusSearch(0.1);   // 法线估计半径 记得改为全局变量
```

若只把 leaf 放大到 1.5 m 而不改法线半径，1.5 m 体素点云上每点在 0.1 m 内几乎无邻居
→ 法线退化为 NaN → FPFH 全零 → 粗配准必然失败。

### 证据 5：粗配准结果无合理性校验

修改前 `rough_match()` 成功即无条件作为 GICP 初值：

```cpp
if (rough_match()) {
  prior = rough_result_t_;   // 关键：用粗配准结果作为细配准初值
}
```

FPFH + RANSAC 在对称 / 空旷场地会给出完全错误的解（实测出现过旋转误差 99°），
而 NIS 可能因为“自洽的错误位姿”误接受。

---

## 根因

粗配准阶段使用**未降采样的全图**（13.5 万点）计算 FPFH 与暴力对应搜索；
同时 YAML 中用于控制粗配准分辨率的参数未被声明、法线半径硬编码，
导致“本想调参提速”的手段全部失效。单次注册从亚秒级劣化到分钟级，
并与 [005](./005-small-gicp-registration-blocks-tf-publish.md) 的回调串行问题叠加，
直接冻结 `map→odom`。

---

## 修改内容

> 工作区当前已修改（相对 `HEAD`），`colcon build --packages-select small_gicp_relocalization` 通过。

### 1. 头文件：新增粗/细两级配准参数成员

`include/small_gicp_relocalization/small_gicp_relocalization.hpp:78-85`

```cpp
// ===== ##2026-10-20## [RELOC-FIX] P2-2: 粗/细两级配准的分辨率与迭代控制 =====
float  coarse_leaf_size_;               // 粗配准源/目标点云体素边长 [m]
float  fine_leaf_size_;                 // 细配准（GICP）源点云体素边长 [m]，<=0 时沿用 registered_leaf_size
int    coarse_max_iterations_;          // 粗配准模式下 GICP 迭代上限
int    fine_max_iterations_;            // 细配准模式下的迭代上限
double coarse_max_translation_delta_;   // 粗配准结果允许的最大平移修正 [m]
double coarse_max_rotation_delta_deg_;  // 粗配准结果允许的最大旋转修正 [deg]
double tf_future_offset_sec_;           // map->odom 时间戳前推量 [s]
```

### 2. 构造函数：声明并读取参数 + 合法性夹紧

`src/small_gicp_relocalization.cpp:71-81`（声明）

```cpp
this->declare_parameter("coarse_leaf_size", 1.5);
this->declare_parameter("fine_leaf_size", 0.0);          // <=0 → 沿用 registered_leaf_size
this->declare_parameter("coarse_max_iterations", 50);
this->declare_parameter("fine_max_iterations", 30);
this->declare_parameter("coarse_max_translation_delta", 8.0);
this->declare_parameter("coarse_max_rotation_delta_deg", 35.0);
this->declare_parameter("tf_future_offset_sec", 0.1);
```

`src/small_gicp_relocalization.cpp:113-131`（读取与夹紧）

```cpp
this->get_parameter("coarse_leaf_size", coarse_leaf_size_);
this->get_parameter("fine_leaf_size", fine_leaf_size_);
...
if (fine_leaf_size_ <= 0.0f) {          // 未配置时保持旧行为
  fine_leaf_size_ = registered_leaf_size_;
}
if (coarse_leaf_size_ < fine_leaf_size_) {   // 粗配准必须更粗
  RCLCPP_WARN(..., "coarse_leaf_size (%.3f) < fine_leaf_size (%.3f), clamping ...");
  coarse_leaf_size_ = fine_leaf_size_;
}
```

### 3. 粗配准目标点云：真正按 `coarse_leaf_size` 降采样

`src/small_gicp_relocalization.cpp:176-190`

```cpp
// 从 target_（已 0.1 m 体素化、点数少 8 倍）再体素化到 coarse_leaf_size
rough_target = small_gicp::voxelgrid_sampling_omp<
  pcl::PointCloud<pcl::PointCovariance>, pcl::PointCloud<pcl::PointXYZ>>(
  *target_, coarse_leaf_size_, num_threads_);
RCLCPP_INFO(this->get_logger(),
  "Coarse registration target: %zu -> %zu points (leaf %.2f m)",
  target_->size(), rough_target->size(), coarse_leaf_size_);
```

### 4. 法线 / FPFH 搜索半径随体素自动放大

`src/small_gicp_relocalization.cpp:405-419`

```cpp
const double coarse_normal_radius = 2.0 * static_cast<double>(coarse_leaf_size_);
const double coarse_fpfh_radius   = std::max(fpfh_radius_, 3.0 * static_cast<double>(coarse_leaf_size_));
...
normal_.setRadiusSearch(coarse_normal_radius);
fpfh_.setRadiusSearch(coarse_fpfh_radius);
```

### 5. RANSAC 内点阈值随体素放大

`src/small_gicp_relocalization.cpp:442-447`

```cpp
// 粗体素质心偏差可达约半个体素对角线，沿用 0.5 m 会把正确对应判成外点
const double coarse_inlier_threshold =
  std::max(ransac_inlier_threshold_, 0.6 * static_cast<double>(coarse_leaf_size_));
ransac.setInlierThreshold(coarse_inlier_threshold);
```

### 6. 细配准源点云使用 `fine_leaf_size`

`src/small_gicp_relocalization.cpp:483-486`

```cpp
source_ = small_gicp::voxelgrid_sampling_omp<...>(
  *accumulated_cloud_, fine_leaf_size_);       // 原为 registered_leaf_size_
```

### 7. 粗配准源点云按同一 `coarse_leaf_size` 降采样

`src/small_gicp_relocalization.cpp:505-508`

```cpp
rough_source = small_gicp::voxelgrid_sampling_omp<
  pcl::PointCloud<pcl::PointCovariance>, pcl::PointCloud<pcl::PointXYZ>>(
  *source_, coarse_leaf_size_, num_threads_);
```

源/目标必须同一体素尺度，否则两侧 FPFH 描述子尺度不一致，匹配率极低。

### 8. 粗配准结果合理性守卫

`src/small_gicp_relocalization.cpp:514-529`

```cpp
const Eigen::Isometry3d delta = rough_result_t_ * previous_result_t_.inverse();
const double d_trans   = delta.translation().norm();
const double d_rot_deg = Eigen::AngleAxisd(delta.rotation()).angle() * kRadToDeg;
if (d_trans > coarse_max_translation_delta_ || d_rot_deg > coarse_max_rotation_delta_deg_) {
  RCLCPP_WARN(this->get_logger(),
    "[ROUGH] coarse result rejected: delta t=%.2f m (limit %.2f), r=%.1f deg (limit %.1f)", ...);
  prior = previous_result_t_;       // 回退到上一可信位姿
} else {
  prior = rough_result_t_;
}
```

### 9. GICP 迭代上限按模式取值

`src/small_gicp_relocalization.cpp:543`

```cpp
register_->optimizer.max_iterations = coarse_mode ? coarse_max_iterations_ : fine_max_iterations_;
```

原为硬编码 `20`，粗配准初值误差大时经常 `converged=false`。

### 10. `map→odom` 时间戳前推量参数化

`src/small_gicp_relocalization.cpp:574`

```cpp
transform_stamped.header.stamp =
  last_scan_time_ + rclcpp::Duration::from_seconds(tf_future_offset_sec_);   // 原为硬编码 0.1
```

### 11. YAML 参数与注释

`config/reality/nav2_params.yaml:164-201`（节选）

```yaml
global_leaf_size: 0.1
# 细配准源点云体素 [m]：仅在 fine_leaf_size <= 0 时作为回退值生效
registered_leaf_size: 0.1
# FPFH 实际生效值 = max(fpfh_radius, 3 × coarse_leaf_size)
fpfh_radius: 0.25
# RANSAC 实际生效值 = max(ransac_inlier_threshold, 0.6 × coarse_leaf_size)
ransac_inlier_threshold: 0.5
coarse_leaf_size: 1.5
fine_leaf_size: 0.3
coarse_max_iterations: 50
fine_max_iterations: 30
coarse_max_translation_delta: 8.0
coarse_max_rotation_delta_deg: 35.0
tf_future_offset_sec: 0.2
```

---

## 验证（离线）

真实先验地图 + 合成扫描（`registered_scan` 10 Hz）单进程运行节点：

```
[1792506446.928] Coarse registration target: 135825 -> 790 points (leaf 1.50 m)
[1792506469.493] ////Performing registration with 107470 accumulated points ////
[1792506469.521] [ROUGH] RANSAC inliers: 18 / 275
[1792506469.522] [ROUGH] coarse result rejected: delta t=0.82 m (limit 8.00), r=99.2 deg (limit 35.0)
[1792506469.724] [NIS-1] eps_res=6954.0  thr=14480.4
[1792506469.724] [NIS] relocalization ACCEPTED
[1792506469.993] ////Performing registration with 107470 accumulated points ////   ← 下一轮（细配准）
[1792506470.014] [NIS] relocalization ACCEPTED
```

| 指标 | 修改前 | 修改后 |
|---|---:|---:|
| 粗配准目标点数 | 135,596 | **790** |
| FPFH + RANSAC 耗时 | 15.2 s | **0.028 s** |
| 单次 `performRegistration()`（粗配准） | **65.4 s** | **0.230 s** |
| 单次 `performRegistration()`（细配准） | — | **0.020 s** |
| GICP 是否收敛 | `converged=0` | 收敛并 ACCEPTED |
| 新增守卫触发情况 | 无此逻辑 | 正确拒绝旋转误差 99° 的粗配准结果 |

速度提升约 **280 倍**。

---

## 遗留问题与调参建议

1. **本修复不解决 TF 阻塞**：注册仍在 executor 线程内同步执行。
   同一轮实测中 `map→odom` 的最大发布间隔为 **0.280 s**（1733 条心跳中），
   导航在此时刻下发目标仍可能触发 `extrapolation into the future`。
   需按 [005](./005-small-gicp-registration-blocks-tf-publish.md) 方案 A 把重计算移出回调线程。
2. `coarse_leaf_size` 取值权衡：
   - 1.5 m 时 790 点，速度足够但 FPFH 描述子较粗；
   - 若现场出现粗配准匹配率低（`[ROUGH] RANSAC inliers` 偏少），可降到 0.8~1.0 m（约 1,500~3,000 点）再试。
3. 三处派生参数是**隐式耦合**的，改 `coarse_leaf_size` 会同时改变：
   法线半径（2×leaf）、FPFH 半径（3×leaf）、RANSAC 内点阈值（0.6×leaf）。
4. `rough_target` 仍在构造函数中生成一次。若后续支持多张先验地图或地图切换，需要提供重建入口。

---

## 相关文件

- `src/pb2025_sentry_nav/small_gicp_relocalization/src/small_gicp_relocalization.cpp`
  - `:69-82` 参数声明
  - `:113-131` 参数读取与夹紧
  - `:176-190` 粗配准目标点云降采样
  - `:405-419` 法线 / FPFH 半径
  - `:442-447` RANSAC 内点阈值
  - `:483-486` 细配准源点云 `fine_leaf_size`
  - `:505-508` 粗配准源点云降采样
  - `:514-529` 粗配准结果守卫
  - `:543` 迭代上限按模式
  - `:574` `tf_future_offset_sec`
- `src/pb2025_sentry_nav/small_gicp_relocalization/include/small_gicp_relocalization/small_gicp_relocalization.hpp:78-85`
- `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml:164-201`
- `src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/GX_test_show.pcd`（验证用先验地图，1,098,176 点）

---

## 处理记录

| 日期 | 处理人 | 改动 / 结论 |
|------|--------|-------------|
| 2026-10-20 | — | 完成上述 11 处修改；`colcon build --packages-select small_gicp_relocalization` 通过；离线实测粗配准 65.4 s → 0.230 s。待实机复测（重点确认粗配准成功率与 `[ROUGH] RANSAC inliers` 数量） |
