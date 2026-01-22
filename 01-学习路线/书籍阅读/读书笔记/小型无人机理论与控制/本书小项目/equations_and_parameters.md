# MAV六自由度运动方程与参数

本文档记录《小型无人机理论与控制》第三章所需的运动方程和飞行器参数。

## 1. 状态变量定义

| 符号 | 含义 | 单位 |
|------|------|------|
| $p_n, p_e, p_d$ | 北-东-地坐标系下的位置 | m |
| $u, v, w$ | 机体坐标系下的速度分量 | m/s |
| $\phi, \theta, \psi$ | 欧拉角（滚转、俯仰、偏航） | rad |
| $p, q, r$ | 机体坐标系下的角速度分量 | rad/s |

## 2. 运动方程

### 2.1 平移运动学方程（位置微分）

$$
\begin{pmatrix} \dot{p}_n \\ \dot{p}_e \\ \dot{p}_d \end{pmatrix} =
\begin{pmatrix}
c_\theta c_\psi & s_\phi s_\theta c_\psi - c_\phi s_\psi & c_\phi s_\theta c_\psi + s_\phi s_\psi \\
c_\theta s_\psi & s_\phi s_\theta s_\psi + c_\phi c_\psi & c_\phi s_\theta s_\psi - s_\phi c_\psi \\
-s_\theta & s_\phi c_\theta & c_\phi c_\theta
\end{pmatrix}
\begin{pmatrix} u \\ v \\ w \end{pmatrix}
$$

其中 $c$ 表示 $\cos$，$s$ 表示 $\sin$。

这是从机体坐标系到惯性坐标系的旋转矩阵 $R_b^v$。

### 2.2 平移动力学方程（速度微分）

$$
\begin{pmatrix} \dot{u} \\ \dot{v} \\ \dot{w} \end{pmatrix} =
\begin{pmatrix} rv - qw \\ pw - ru \\ qu - pv \end{pmatrix} +
\frac{1}{m} \begin{pmatrix} f_x \\ f_y \\ f_z \end{pmatrix}
$$

其中：
- 第一项是科里奥利力/离心力项
- $f_x, f_y, f_z$ 是机体坐标系下的外力（包括气动力、重力、推力）

### 2.3 姿态运动学方程（欧拉角微分）

$$
\begin{pmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{pmatrix} =
\begin{pmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi\sec\theta & \cos\phi\sec\theta
\end{pmatrix}
\begin{pmatrix} p \\ q \\ r \end{pmatrix}
$$

注意：当 $\theta = \pm 90°$ 时存在奇异点。

### 2.4 转动动力学方程（角速度微分）

$$
\begin{pmatrix} \dot{p} \\ \dot{q} \\ \dot{r} \end{pmatrix} =
\begin{pmatrix}
\Gamma_1 pq - \Gamma_2 qr \\
\Gamma_5 pr - \Gamma_6(p^2 - r^2) \\
\Gamma_7 pq - \Gamma_1 qr
\end{pmatrix} +
\begin{pmatrix}
\Gamma_3 l + \Gamma_4 n \\
\frac{1}{J_y} m \\
\Gamma_4 l + \Gamma_8 n
\end{pmatrix}
$$

其中 $l, m, n$ 是机体坐标系下的力矩分量（滚转力矩、俯仰力矩、偏航力矩）。

### 2.5 Gamma参数定义

$$
\Gamma \triangleq J_x J_z - J_{xz}^2
$$

$$
\begin{cases}
\Gamma_1 = \dfrac{J_{xz}(J_x - J_y + J_z)}{\Gamma} \\[10pt]
\Gamma_2 = \dfrac{J_z(J_z - J_y) + J_{xz}^2}{\Gamma} \\[10pt]
\Gamma_3 = \dfrac{J_z}{\Gamma} \\[10pt]
\Gamma_4 = \dfrac{J_{xz}}{\Gamma} \\[10pt]
\Gamma_5 = \dfrac{J_z - J_x}{J_y} \\[10pt]
\Gamma_6 = \dfrac{J_{xz}}{J_y} \\[10pt]
\Gamma_7 = \dfrac{(J_x - J_y)J_x + J_{xz}^2}{\Gamma} \\[10pt]
\Gamma_8 = \dfrac{J_x}{\Gamma}
\end{cases}
$$

## 3. 飞行器参数

### 3.1 Zagi飞翼参数（表E.1）

#### 物理参数

| 参数 | 值 | 说明 |
|------|------|------|
| $m$ | 1.56 kg | 质量 |
| $J_x$ | 0.1147 kg·m² | x轴转动惯量 |
| $J_y$ | 0.0576 kg·m² | y轴转动惯量 |
| $J_z$ | 0.1712 kg·m² | z轴转动惯量 |
| $J_{xz}$ | 0.0015 kg·m² | 惯性积 |
| $S$ | 0.2589 m² | 机翼面积 |
| $b$ | 1.4224 m | 翼展 |
| $c$ | 0.3302 m | 平均气动弦长 |
| $S_{prop}$ | 0.0314 m² | 螺旋桨面积 |
| $\rho$ | 1.2682 kg/m³ | 空气密度 |
| $k_{motor}$ | 20 | 电机常数 |
| $k_{T_p}$ | 0 | 推力常数 |
| $k_\Omega$ | 0 | 角速度常数 |
| $e$ | 0.9 | Oswald效率因子 |

#### 纵向气动系数

| 参数 | 值 | 参数 | 值 |
|------|------|------|------|
| $C_{L_0}$ | 0.09167 | $C_{L_\alpha}$ | 3.5016 |
| $C_{D_0}$ | 0.01631 | $C_{D_\alpha}$ | 0.2108 |
| $C_{m_0}$ | -0.02338 | $C_{m_\alpha}$ | -0.5675 |
| $C_{L_q}$ | 2.8932 | $C_{D_q}$ | 0 |
| $C_{m_q}$ | -1.3990 | $C_{L_{\delta_e}}$ | 0.2724 |
| $C_{D_{\delta_e}}$ | 0.3045 | $C_{m_{\delta_e}}$ | -0.3254 |
| $C_{prop}$ | 1.0 | $M$ | 50 |
| $\alpha_0$ | 0.4712 | $\epsilon$ | 0.1592 |
| $C_{D_p}$ | 0.0254 | | |

#### 横向气动系数

| 参数 | 值 | 参数 | 值 |
|------|------|------|------|
| $C_{Y_0}$ | 0 | $C_{l_0}$ | 0 |
| $C_{n_0}$ | 0 | $C_{Y_\beta}$ | -0.07359 |
| $C_{l_\beta}$ | -0.02854 | $C_{n_\beta}$ | 0.00040 |
| $C_{Y_p}$ | 0 | $C_{l_p}$ | -0.3209 |
| $C_{n_p}$ | -0.01297 | $C_{Y_r}$ | 0 |
| $C_{l_r}$ | 0.03066 | $C_{n_r}$ | -0.00434 |
| $C_{Y_{\delta_a}}$ | 0 | $C_{l_{\delta_a}}$ | 0.1682 |
| $C_{n_{\delta_a}}$ | -0.00328 | | |

### 3.2 无人机参数（表E.2）

#### 物理参数

| 参数 | 值 | 说明 |
|------|------|------|
| $m$ | 13.5 kg | 质量 |
| $J_x$ | 0.8244 kg·m² | x轴转动惯量 |
| $J_y$ | 1.135 kg·m² | y轴转动惯量 |
| $J_z$ | 1.759 kg·m² | z轴转动惯量 |
| $J_{xz}$ | 0.1204 kg·m² | 惯性积 |
| $S$ | 0.55 m² | 机翼面积 |
| $b$ | 2.8956 m | 翼展 |
| $c$ | 0.18994 m | 平均气动弦长 |
| $S_{prop}$ | 0.2027 m² | 螺旋桨面积 |
| $\rho$ | 1.2682 kg/m³ | 空气密度 |
| $k_{motor}$ | 80 | 电机常数 |
| $k_{T_p}$ | 0 | 推力常数 |
| $k_\Omega$ | 0 | 角速度常数 |
| $e$ | 0.9 | Oswald效率因子 |

#### 纵向气动系数

| 参数 | 值 | 参数 | 值 |
|------|------|------|------|
| $C_{L_0}$ | 0.28 | $C_{L_\alpha}$ | 3.45 |
| $C_{D_0}$ | 0.03 | $C_{D_\alpha}$ | 0.3 |
| $C_{m_0}$ | -0.02338 | $C_{m_\alpha}$ | -0.38 |
| $C_{L_q}$ | 0 | $C_{D_q}$ | 0 |
| $C_{m_q}$ | -3.6 | $C_{L_{\delta_e}}$ | -0.36 |
| $C_{D_{\delta_e}}$ | 0 | $C_{m_{\delta_e}}$ | -0.5 |
| $C_{prop}$ | 1.0 | $M$ | 50 |
| $\alpha_0$ | 0.4712 | $\epsilon$ | 0.1592 |
| $C_{D_p}$ | 0.0437 | $C_{n_{\delta_e}}$ | -0.032 |

#### 横向气动系数

| 参数 | 值 | 参数 | 值 |
|------|------|------|------|
| $C_{Y_0}$ | 0 | $C_{l_0}$ | 0 |
| $C_{n_0}$ | 0 | $C_{Y_\beta}$ | -0.98 |
| $C_{l_\beta}$ | -0.12 | $C_{n_\beta}$ | 0.25 |
| $C_{Y_p}$ | 0 | $C_{l_p}$ | -0.26 |
| $C_{n_p}$ | 0.022 | $C_{Y_r}$ | 0 |
| $C_{l_r}$ | 0.14 | $C_{n_r}$ | -0.35 |
| $C_{Y_{\delta_a}}$ | 0 | $C_{l_{\delta_a}}$ | 0.08 |
| $C_{n_{\delta_a}}$ | 0.06 | $C_{Y_{\delta_r}}$ | -0.17 |
| $C_{l_{\delta_r}}$ | 0.105 | | |

## 4. 第三章设计任务

1. **实现运动方程**：用代码实现式(3.14)~(3.17)给出的MAV运动方程
   - 输入：机体坐标系下的力 $(f_x, f_y, f_z)$ 和力矩 $(l, m, n)$
   - 参数：质量、惯量、惯积、初始状态

2. **连接动画模块**：与第2章的3D线框动画连接

3. **验证运动方程**：通过单独设置每个力和力矩为非零值验证正确性

4. **验证耦合效应**：
   - 设置 $J_{xz}=0$，验证滚转和偏航无耦合
   - 设置 $J_{xz}\neq 0$，验证滚转和偏航有耦合
