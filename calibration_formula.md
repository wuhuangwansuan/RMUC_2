# 双 MID360 外参标定公式整理

本文记录 RMUC_2 工程双 MID360 外参标定过程中使用的全部数学工具，按「基础 → 应用」顺序排列，并附本次标定的实际数值。

标定任务分两步：

1. **两雷达间外参** `T_right_lidar ← left_lidar`（159 在 151 系下）
2. **主雷达与底盘外参** `T_base_footprint ← right_lidar`（151 相对 base_footprint）

涉及的坐标系：

| 名称 | 含义 |
|---|---|
| `base_footprint` | 底盘坐标系，原点在地面投影，z 轴竖直向上 |
| `right_lidar` | 主雷达 151（192.168.1.151）光学坐标系，倾斜安装 |
| `left_lidar` | 副雷达 159（192.168.1.159）光学坐标系，倾斜安装 |
| `imu_link` | 151 内置 IMU 坐标系，与 `right_lidar` 只差平移 |

---

## 1. 记号约定

- 齐次变换：

$$T_{A\leftarrow B}=\begin{bmatrix}R_{AB}&\mathbf t_{AB}\\ \mathbf 0^\top&1\end{bmatrix}$$

含义是**把 B 系的点表达到 A 系**。

- 向量默认列向量；$|\mathbf n|=1$ 表示单位向量
- $R$ 为 3×3 旋转矩阵（正交，$\det R=1$）
- 简写：$c_x\equiv\cos x,\ s_x\equiv\sin x$

---

## 2. 旋转矩阵与欧拉角

### 2.1 合成顺序

ROS（REP-103）的 rpy 是 **ZYX 内旋**，等价于 XYZ 外旋，合成顺序为「先 roll，再 pitch，后 yaw」：

$$R=R_z(\text{yaw})\,R_y(\text{pitch})\,R_x(\text{roll})$$

三个基本旋转矩阵：

$$R_x(a)=\begin{bmatrix}1&0&0\\0&\cos a&-\sin a\\0&\sin a&\cos a\end{bmatrix},\quad
R_y(b)=\begin{bmatrix}\cos b&0&\sin b\\0&1&0\\-\sin b&0&\cos b\end{bmatrix},\quad
R_z(c)=\begin{bmatrix}\cos c&-\sin c&0\\\sin c&\cos c&0\\0&0&1\end{bmatrix}$$

### 2.2 展开形式

$$R=\begin{bmatrix}
c_yc_p & c_ys_ps_r-s_yc_r & c_ys_pc_r+s_ys_r\\
s_yc_p & s_ys_ps_r+c_yc_r & s_ys_pc_r-c_ys_r\\
-s_p   & c_ps_r          & c_pc_r
\end{bmatrix}$$

### 2.3 反解欧拉角

$$\text{pitch}=\operatorname{atan2}\!\big(-R_{20},\ \sqrt{R_{00}^2+R_{10}^2}\big)$$

$$\text{roll}=\operatorname{atan2}(R_{21},R_{22})$$

$$\text{yaw}=\operatorname{atan2}(R_{10},R_{00})$$

### 2.4 关键性质

右下角元素

$$\boxed{\,R_{22}=\cos(\text{pitch})\cos(\text{roll})\,}$$

**不含 yaw**。这直接说明：

- 子坐标系 z 轴与父坐标系 z 轴的夹角 $=\arccos(R_{22})$，与 yaw 无关
- 因此「绕父系竖直轴旋转」不可能改变倾斜量（roll/pitch）

### 2.5 与四元数的转换

$$q=(w,x,y,z)=\Big(\cos\tfrac\theta2,\ \mathbf k\sin\tfrac\theta2\Big)$$

旋转矩阵转四元数（取数值稳定分支），记 $\operatorname{tr}=R_{00}+R_{11}+R_{22}$，若 $\operatorname{tr}>0$：

$$s=2\sqrt{1+\operatorname{tr}},\quad
w=\tfrac{s}{4},\quad
x=\frac{R_{21}-R_{12}}{s},\quad
y=\frac{R_{02}-R_{20}}{s},\quad
z=\frac{R_{10}-R_{01}}{s}$$

否则按 $R_{00},R_{11},R_{22}$ 三者最大者选对应分支。

---

## 3. 齐次变换

### 3.1 点的变换

$$\mathbf p_A=R_{AB}\,\mathbf p_B+\mathbf t_{AB}$$

### 3.2 复合（链式相乘）

$$T_{A\leftarrow C}=T_{A\leftarrow B}\,T_{B\leftarrow C}$$

### 3.3 求逆（闭式）

$$T_{A\leftarrow B}^{-1}=\begin{bmatrix}R_{AB}^\top&-R_{AB}^\top\mathbf t_{AB}\\ \mathbf 0^\top&1\end{bmatrix}$$

---

## 4. 两雷达间外参

### 4.1 兄弟关系推导

已知两条到同一父系的变换，反推两者之间的关系：

$$\boxed{\,T_{\text{base}\leftarrow\text{left}}=T_{\text{base}\leftarrow\text{right}}\;T_{\text{right}\leftarrow\text{left}}\,}$$

$$\Rightarrow\quad T_{\text{right}\leftarrow\text{left}}=T_{\text{base}\leftarrow\text{right}}^{-1}\,T_{\text{base}\leftarrow\text{left}}$$

### 4.2 多次标定取平均

旋转不能用欧拉角直接平均（受万向锁与 $2\pi$ 周期影响），改用四元数：

1. 各次旋转转四元数 $q_i$
2. 半球对齐：若 $q_i\cdot q_0<0$ 则取 $-q_i$
3. 算术平均并归一化

$$q_{\text{avg}}=\frac{\sum_i q_i}{\left\|\sum_i q_i\right\|}$$

本次两次标定角度几乎相同，与直接平均欧拉角的差异 $<10^{-7}$ rad。

---

## 5. 绕任意轴旋转

绕**过点 $\mathbf p$、方向 $\mathbf k$（单位向量）、角度 $\theta$** 的旋转变换：

$$\boxed{\,M=\mathrm{Trans}(\mathbf p)\,R_{\mathbf k}(\theta)\,\mathrm{Trans}(-\mathbf p)
=\begin{bmatrix}R_{\mathbf k}(\theta)&\mathbf p-R_{\mathbf k}(\theta)\,\mathbf p\\ \mathbf 0^\top&1\end{bmatrix}}$$

其中 $R_{\mathbf k}(\theta)$ 用 **Rodrigues 公式**：

$$R_{\mathbf k}(\theta)=I+\sin\theta\,[\mathbf k]_\times+(1-\cos\theta)\,[\mathbf k]_\times^2$$

$$[\mathbf k]_\times=\begin{bmatrix}0&-k_3&k_2\\k_3&0&-k_1\\-k_2&k_1&0\end{bmatrix}$$

### 5.1 绕竖直轴旋转的恒等式

$$R_z(\theta)\,R_z(\text{yaw})R_y(\text{pitch})R_x(\text{roll})
=R_z(\text{yaw}+\theta)\,R_y(\text{pitch})R_x(\text{roll})$$

因为 $R_z(\theta)R_z(\text{yaw})=R_z(\text{yaw}+\theta)$，而矩阵乘法满足结合律。

**结论**：绕 base z 轴转 $\theta$ ⟺ yaw 加 $\theta$，roll/pitch 严格不变。

### 5.2 本次应用

159 相对 151 的绕竖直轴修正 $\theta=+3°$、后来改为 $+5°$，圆心为 151 的位置：

$$\mathbf p=(-0.13,-0.13,0.4),\qquad \mathbf k=(0,0,1),\qquad T_{\text{new}}=M\,T_{\text{old}}$$

只有 $x,y,\text{yaw}$ 变化（沿圆周运动），$z,\text{roll},\text{pitch}$ 为不变量。

---

## 6. 平面拟合

### 6.1 RANSAC（粗定位）

随机抽 3 点定平面，统计内点（距离小于阈值）数量，迭代取内点最多者。

三点定平面：

$$\mathbf n=\frac{(\mathbf p_1-\mathbf p_0)\times(\mathbf p_2-\mathbf p_0)}{\left\|(\mathbf p_1-\mathbf p_0)\times(\mathbf p_2-\mathbf p_0)\right\|},\qquad d=-\mathbf n\cdot\mathbf p_0$$

点到平面距离：

$$\text{dist}(\mathbf p)=|\mathbf n\cdot\mathbf p+d|$$

### 6.2 SVD 最小二乘精修

对平面内点集 $\{\mathbf p_i\}$：

1. 质心 $\bar{\mathbf p}=\frac1N\sum_i\mathbf p_i$
2. 中心化 $\mathbf q_i=\mathbf p_i-\bar{\mathbf p}$，堆成矩阵 $Q=[\mathbf q_1;\dots;\mathbf q_N]$
3. 奇异值分解 $Q=U\Sigma V^\top$
4. **法向 = 最小奇异值对应的右奇异向量** $\mathbf n=V_{:,3}$

原理：使 $\sum_i\|\mathbf q_i\|^2$ 沿法向的分量最小，即最小化点到平面的平方距离和。

**点到平面的垂直距离与残差**：

$$\boxed{\,h=\big|\mathbf n\cdot\bar{\mathbf p}\big|\,},\qquad r_i=\mathbf n\cdot(\mathbf p_i-\bar{\mathbf p})$$

### 6.3 局部法向（PCA）

对点的 $k$ 个近邻 $\{\mathbf q_i\}$：

$$C=\frac1k\sum_i(\mathbf q_i-\bar{\mathbf q})(\mathbf q_i-\bar{\mathbf q})^\top,\qquad C=V\Lambda V^\top,\quad \lambda_1\le\lambda_2\le\lambda_3$$

- **法向**：$\mathbf n=\mathbf v_1$（最小特征值对应特征向量）
- **平面度**：$\lambda_1/\lambda_2$，越小越平（阈值取 0.1 以剔除墙角与边缘点）

---

## 7. 地面标定方程

### 7.1 建模

设地面在雷达系中为

$$\mathbf n^\top\mathbf p_{\text{radar}}=-h$$

其中 $\mathbf n$ 为指向天空的单位法向，$h>0$ 是雷达到地面的垂直距离。

代入 $\mathbf p_{\text{radar}}=R^\top(\mathbf p_{\text{base}}-\mathbf t)$（$T_{\text{base}\leftarrow\text{radar}}=[R\mid\mathbf t]$）：

$$\mathbf n^\top R^\top(\mathbf p_{\text{base}}-\mathbf t)=-h$$

$$\Longrightarrow\quad (R\mathbf n)^\top\mathbf p_{\text{base}}=(R\mathbf n)^\top\mathbf t-h$$

### 7.2 标定条件

`base_footprint` 定义为「地面上的投影点，z 轴竖直」，即地面在 base 系中必须是 $\mathbf e_z^\top\mathbf p=0$ 平面。对照上式得**两条独立约束**：

$$\boxed{\,R\,\mathbf n=\mathbf e_z\,}\qquad\qquad \boxed{\,\mathbf t_z=h\,}$$

| 约束 | 方程数 | 独立约束 | 可确定自由度 |
|---|---|---|---|
| $R\mathbf n=\mathbf e_z$ | 3 | **2**（因 $\lvert\mathbf n\rvert=\lvert\mathbf e_z\rvert=1$） | roll, pitch |
| $\mathbf t_z=h$ | 1 | **1** | 高度 z |

**不可确定**：yaw 与 $\mathbf t_x,\mathbf t_y$ 共 3 个自由度（绕竖直轴旋转不动 $\mathbf e_z$）。

---

## 8. 求解方法

### 8.1 解析解（固定 yaw）

设 $R=R_z(y)R_y(p)R_x(r)$。由 $R\mathbf n=\mathbf e_z$：

$$R_z(y)R_y(p)R_x(r)\mathbf n=\mathbf e_z
\;\xrightarrow{\ R_z(-y)\mathbf e_z=\mathbf e_z\ }\;
R_y(p)R_x(r)\mathbf n=\mathbf e_z$$

记 $\mathbf m=\mathbf n=(m_x,m_y,m_z)$。先算 $R_x(r)\mathbf m$ 并令其 $y$ 分量为零：

$$m_y\cos r-m_z\sin r=0\ \Longrightarrow\ \boxed{\,r=\operatorname{atan2}(m_y,m_z)\,}$$

此时

$$R_x(r)\mathbf m=\big(m_x,\ 0,\ \sqrt{m_y^2+m_z^2}\big)$$

再由 $R_y(p)$ 把 $x$ 分量转到零：

$$\boxed{\,p=\operatorname{atan2}\!\big(-m_x,\ \sqrt{m_y^2+m_z^2}\big)\,}$$

自洽性检验：第三分量 $=\sqrt{m_x^2+m_y^2+m_z^2}=|\mathbf m|=1$ ✅

### 8.2 最小旋转法

把向量 $\mathbf a$ 转到 $\mathbf b$ 的最小旋转：

$$\mathbf v=\mathbf a\times\mathbf b,\qquad c=\mathbf a\cdot\mathbf b$$

$$R=I+[\mathbf v]_\times+\frac{[\mathbf v]_\times^2}{1+c}$$

取 $\mathbf a=R_{\text{old}}\mathbf n$、$\mathbf b=\mathbf e_z$，得 $R_{\text{new}}=R_{\text{fix}}R_{\text{old}}$。

**两方案对比**：roll/pitch 完全相同（差 $0.00000°$），仅 yaw 差 $0.37004°$。最小旋转会污染未标定的 yaw，故最终采用解析解方案。

---

## 9. 轴角提取（验证用）

$$\theta=\arccos\!\left(\frac{\operatorname{tr}(R)-1}{2}\right)$$

$$\mathbf k=\frac{1}{2\sin\theta}\begin{bmatrix}R_{21}-R_{12}\\ R_{02}-R_{20}\\ R_{10}-R_{01}\end{bmatrix}$$

用于验证「实际施加的变换是否为纯竖直轴旋转」。

---

## 10. 几何一致性判据

| 关系 | 判据 | 本次实测 |
|---|---|---|
| 两平面平行 | $\lvert\mathbf n_1\cdot\mathbf n_2\rvert\approx1$ | 地板 vs 天花板：**0.097°** |
| 两平面正交 | $\mathbf n_1\cdot\mathbf n_2\approx0$ | 地板 vs 墙：**89.57°** |
| 墙竖直 | $\mathbf n_{\text{wall}}\perp\mathbf g$，其中 $\mathbf g=-\mathbf n_{\text{floor}}$ | 偏差 **0.55°** |
| 坐标系判定 | $\angle(\mathbf n_{\text{floor}},\mathbf e_z)$ ≈ 雷达安装倾角 | 61.64° vs 60.55° |
| 纯竖直轴旋转 | 轴角提取 $\mathbf k=(0,0,1)$ | ✅ |

---

## 11. 本次标定数值汇总

### 11.1 数据源

`GX_test_show.pcd` 为 **right_lidar 系**点云（非世界系），判定依据：

- 地面法向与点云 z 轴夹角 **61.64°** ≈ 雷达安装倾角 **60.55°**
- 墙面法向与重力方向正交（偏差 0.55°）
- 原点附近 0.3 m 内仅 57 点（雷达盲区特征）

### 11.2 拟合结果

```
地板法向 n_f = [-0.807707, +0.351012, +0.473709]    （雷达系）
雷达离地 h_f = 0.38952 m
房间净高     = 4.0128 m = 0.38952（地板）+ 3.62325（天花板）

旧外参倾斜误差 = arccos(R_old · n_f) = 1.8916°
```

### 11.3 标定结果

```
T_base_footprint ← right_lidar (151)
    xyz = [-0.13, -0.13, 0.38952] m
    rpy = [0.637708, 0.940252, -1.0036] rad = [36.5379°, 53.8725°, -57.5020°]

T_right_lidar ← left_lidar (锁定值, 含 +5° 修正)
    xyz = [-0.190548, -0.020410, -0.297065] m
    rpy = [178.543275°, 62.087323°, -0.894927°]

T_base_footprint ← left_lidar (同步)
    xyz = [-0.16387, 0.22184, 0.39554] m
    rpy = [-0.62338, 0.92601, 1.09175] rad
```

### 11.4 验证结果

| 判据 | 旧外参 | 新外参 |
|---|---|---|
| 地板点 base z 均值 | +0.0342 m | **-0.0000 m** |
| 地板点 base z 标准差 | 0.0424 m | **0.0149 m** |
| 天花板 base z | — | **+4.0121 m**（期望 4.0128） |
| 地面法向 → base | 偏 1.89° | **0.000026°** |
| 两雷达关系保持误差 | — | **6e-6 m / 5e-4 deg** |

---

## 12. 约束与注意事项

1. **`T_right_lidar ← left_lidar` 为锁定值**：任何情况下不应重新手调。若需修正两雷达对齐，改 `base_footprint → right_lidar` 的 yaw，再同步重算 159。

2. **151 与 159 必须联动**：

$$T_{\text{base}\leftarrow\text{left}}=T_{\text{base}\leftarrow\text{right}}\,T_{\text{right}\leftarrow\text{left}}$$

每次修改 `base_footprint → right_lidar`，必须用上式重算 `left_lidar` 的 TF，否则两雷达点云会重新错开。

3. **驱动侧外参保持为 0**：`MID360_config.json` 与 `mid360_user_config.json` 中 159 的 `extrinsic_parameter` 必须全为 0。因为 FAST-LIO 已通过 TF 完成 159→151 的变换，若驱动再变换一次会导致**双重变换**。

4. **尚未标定的自由度**：`base_footprint → right_lidar` 的 `x`、`y`（横向位置，需机械测量）与 `yaw`（车头朝向参考），地面拟合无法确定。

---

## 13. 参考文件

| 文件 | 作用 |
|---|---|
| `src/pb2025_sentry_nav/livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py` | 发布三条静态 TF（151 / 159 / imu_link） |
| `src/pb2025_sentry_nav/FAST_LIO/src/laserMapping.cpp` | 双雷达融合，查 `right_lidar ← left_lidar` |
| `src/pb2025_sentry_nav/FAST_LIO/config/mid360.yaml` | FAST-LIO 双雷达配置 |
| `src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/GX_test_show.pcd` | 标定数据源（right_lidar 系点云） |
| `src/pb2025_sentry_nav/livox_ros_driver2/src/comm/pub_handler.cpp` | 驱动侧外参补偿逻辑 |
