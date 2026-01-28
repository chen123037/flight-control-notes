# 第四章 力与力矩模型 - 公式汇总

本文档记录《小型无人机理论与控制》第四章所有力与力矩相关的数学公式。

---

## 1. 设计要求 (4.6节)

1. 实现重力、空气动力、推力和扭力的模块 `forces_moments`，使用附录E中的参数
2. 修改干扰风模块，输出：
   - 机体坐标系的力和力矩
   - 空速 $V_a$、攻角 $\alpha$、侧滑角 $\beta$
   - 惯性坐标系下的风速矢量 $(w_n, w_e, w_d)^T$
3. 通过设置控制面变化为不同的值来验证仿真

---

## 2. 符号定义

### 2.1 状态变量
| 符号 | 含义 | 单位 |
|------|------|------|
| $u, v, w$ | 机体坐标系速度分量 | m/s |
| $p, q, r$ | 机体坐标系角速度分量 | rad/s |
| $\phi, \theta, \psi$ | 欧拉角（滚转、俯仰、偏航） | rad |
| $V_a$ | 空速（相对于空气的速度大小） | m/s |
| $\alpha$ | 攻角（迎角） | rad |
| $\beta$ | 侧滑角 | rad |

### 2.2 控制输入
| 符号 | 含义 | 范围 |
|------|------|------|
| $\delta_e$ | 升降舵偏转角 | rad |
| $\delta_a$ | 副翼偏转角 | rad |
| $\delta_r$ | 方向舵偏转角 | rad |
| $\delta_t$ | 油门（推力）设置 | 0~1 |

### 2.3 几何和物理参数
| 符号 | 含义 | 单位 |
|------|------|------|
| $m$ | 质量 | kg |
| $g$ | 重力加速度 | m/s² |
| $\rho$ | 空气密度 | kg/m³ |
| $S$ | 机翼面积 | m² |
| $b$ | 翼展 | m |
| $c$ | 平均气动弦长 | m |
| $S_{prop}$ | 螺旋桨面积 | m² |
| $AR$ | 展弦比，$AR \triangleq b^2/S$ | - |
| $e$ | 奥斯瓦尔德效率因数 (0.8~1.0) | - |

---

## 3. 作用在MAV上的总力 (式4.18)

$$
\begin{pmatrix} f_x \\ f_y \\ f_z \end{pmatrix} =
\underbrace{\begin{pmatrix} -mg\sin\theta \\ mg\cos\theta\sin\phi \\ mg\cos\theta\cos\phi \end{pmatrix}}_{\text{重力}}
+ \underbrace{\frac{1}{2}\rho V_a^2 S \begin{pmatrix}
C_X(\alpha) + C_{X_q}(\alpha)\frac{c}{2V_a}q + C_{X_{\delta_e}}(\alpha)\delta_e \\
C_{Y_0} + C_{Y_\beta}\beta + C_{Y_p}\frac{b}{2V_a}p + C_{Y_r}\frac{b}{2V_a}r + C_{Y_{\delta_a}}\delta_a + C_{Y_{\delta_r}}\delta_r \\
C_Z(\alpha) + C_{Z_q}(\alpha)\frac{c}{2V_a}q + C_{Z_{\delta_e}}(\alpha)\delta_e
\end{pmatrix}}_{\text{气动力}}
+ \underbrace{\frac{1}{2}\rho S_{prop} C_{prop} \begin{pmatrix} (k_{motor}\delta_t)^2 - V_a^2 \\ 0 \\ 0 \end{pmatrix}}_{\text{推力}}
$$

### 3.1 纵向气动系数 $C_X$ 和 $C_Z$ 的定义 (式4.19)

这些系数将升力/阻力系数从稳定坐标系转换到机体坐标系：

$$
\begin{cases}
C_X(\alpha) \triangleq -C_D(\alpha)\cos\alpha + C_L(\alpha)\sin\alpha \\[5pt]
C_{X_q}(\alpha) \triangleq -C_{D_q}(\alpha)\cos\alpha + C_{L_q}(\alpha)\sin\alpha \\[5pt]
C_{X_{\delta_e}}(\alpha) \triangleq -C_{D_{\delta_e}}(\alpha)\cos\alpha + C_{L_{\delta_e}}(\alpha)\sin\alpha \\[5pt]
C_Z(\alpha) \triangleq -C_D(\alpha)\sin\alpha - C_L(\alpha)\cos\alpha \\[5pt]
C_{Z_q}(\alpha) \triangleq -C_{D_q}(\alpha)\sin\alpha - C_{L_q}(\alpha)\cos\alpha \\[5pt]
C_{Z_{\delta_e}}(\alpha) \triangleq -C_{D_{\delta_e}}(\alpha)\sin\alpha - C_{L_{\delta_e}}(\alpha)\cos\alpha
\end{cases}
$$

**说明**：下标 $X$ 和 $Z$ 表示这些力是作用在机体坐标系的 $X$ 和 $Z$ 方向，对应于 $\mathbf{i}^b$ 和 $\mathbf{k}^b$ 矢量的方向。

---

## 4. 作用在MAV上的总扭矩

$$
\begin{pmatrix} l \\ m \\ n \end{pmatrix} =
\frac{1}{2}\rho V_a^2 S \begin{pmatrix}
b\left[C_{l_0} + C_{l_\beta}\beta + C_{l_p}\frac{b}{2V_a}p + C_{l_r}\frac{b}{2V_a}r + C_{l_{\delta_a}}\delta_a + C_{l_{\delta_r}}\delta_r\right] \\[8pt]
c\left[C_{m_0} + C_{m_\alpha}\alpha + C_{m_q}\frac{c}{2V_a}q + C_{m_{\delta_e}}\delta_e\right] \\[8pt]
b\left[C_{n_0} + C_{n_\beta}\beta + C_{n_p}\frac{b}{2V_a}p + C_{n_r}\frac{b}{2V_a}r + C_{n_{\delta_a}}\delta_a + C_{n_{\delta_r}}\delta_r\right]
\end{pmatrix}
+ \begin{pmatrix} -k_{T_p}(k_\Omega\delta_t)^2 \\ 0 \\ 0 \end{pmatrix}
$$

**说明**：
- $l$ = 滚转力矩（绕 $x$ 轴）
- $m$ = 俯仰力矩（绕 $y$ 轴）
- $n$ = 偏航力矩（绕 $z$ 轴）
- 最后一项是螺旋桨产生的扭矩

---

## 5. 升力系数 $C_L(\alpha)$

### 5.1 完整非线性模型 (式4.9)

$$
C_L(\alpha) = (1 - \sigma(\alpha))(C_{L_0} + C_{L_\alpha}\alpha) + \sigma(\alpha)\left[2\text{sign}(\alpha)\sin^2\alpha\cos\alpha\right]
$$

**物理意义**：
- 第一项：线性升力模型（小攻角时有效）
- 第二项：失速后的平板模型（大攻角时有效）
- $\sigma(\alpha)$：混合函数，实现两种模型的平滑过渡

### 5.2 Sigmoid混合函数 (式4.10)

$$
\sigma(\alpha) = \frac{1 + e^{-M(\alpha - \alpha_0)} + e^{M(\alpha + \alpha_0)}}{(1 + e^{-M(\alpha - \alpha_0)})(1 + e^{M(\alpha + \alpha_0)})}
$$

**参数说明**：
- $M$ = 转换速率（正常数，值越大过渡越陡峭）
- $\alpha_0$ = 失速攻角（分界点，正常数）
- 当 $|\alpha| < \alpha_0$ 时，$\sigma(\alpha) \approx 0$（线性模型）
- 当 $|\alpha| > \alpha_0$ 时，$\sigma(\alpha) \approx 1$（平板模型）

### 5.3 线性升力系数 $C_{L_\alpha}$

$$
C_{L_\alpha} = \frac{\pi \cdot AR}{1 + \sqrt{1 + (AR/2)^2}}
$$

式中：$AR \triangleq b^2/S$ 为机翼的展弦比

### 5.4 简化线性模型 (式4.12)

$$
C_L(\alpha) = C_{L_0} + C_{L_\alpha}\alpha
$$

---

## 6. 阻力系数 $C_D(\alpha)$

### 6.1 完整模型 (式4.11)

$$
C_D(\alpha) = C_{D_p} + \frac{(C_{L_0} + C_{L_\alpha}\alpha)^2}{\pi e \cdot AR}
$$

**组成**：
- $C_{D_p}$ = 废阻力系数（寄生阻力，与攻角无关）
- 第二项 = 诱导阻力（与升力平方成正比）
- $e$ = 奥斯瓦尔德效率因数 (0.8~1.0)

### 6.2 简化线性模型 (式4.13)

$$
C_D(\alpha) = C_{D_0} + C_{D_\alpha}\alpha
$$

---

## 7. 俯仰力矩系数 $C_m(\alpha)$

### 线性模型

$$
C_m(\alpha) = C_{m_0} + C_{m_\alpha}\alpha
$$

**稳定性条件**：$C_{m_\alpha} < 0$ 表明机体俯仰稳定。

---

## 8. 横向力和力矩的线性模型

### 8.1 横向力 (式4.14)

$$
f_y \approx \frac{1}{2}\rho V_a^2 S \left( C_{Y_0} + C_{Y_\beta}\beta + C_{Y_p}\frac{b}{2V_a}p + C_{Y_r}\frac{b}{2V_a}r + C_{Y_{\delta_a}}\delta_a + C_{Y_{\delta_r}}\delta_r \right)
$$

### 8.2 滚转力矩 (式4.15)

$$
l \approx \frac{1}{2}\rho V_a^2 S b \left( C_{l_0} + C_{l_\beta}\beta + C_{l_p}\frac{b}{2V_a}p + C_{l_r}\frac{b}{2V_a}r + C_{l_{\delta_a}}\delta_a + C_{l_{\delta_r}}\delta_r \right)
$$

### 8.3 偏航力矩 (式4.16)

$$
n \approx \frac{1}{2}\rho V_a^2 S b \left( C_{n_0} + C_{n_\beta}\beta + C_{n_p}\frac{b}{2V_a}p + C_{n_r}\frac{b}{2V_a}r + C_{n_{\delta_a}}\delta_a + C_{n_{\delta_r}}\delta_r \right)
$$

**说明**：这些力和力矩都与飞机机体轴一致，故在运动方程式中不需要旋转变换。

---

## 9. 空气动力学系数分类

### 9.1 稳定导数（Stability Derivatives）

这些系数决定了MAV的静态稳定性：

| 系数 | 名称 | 稳定性条件 | 物理意义 |
|------|------|------------|----------|
| $C_{m_\alpha}$ | 俯仰阻尼导数 | $< 0$ | 俯仰稳定 |
| $C_{l_\beta}$ | 滚转稳定性导数 | $< 0$ | 滚转稳定，减小侧滑 |
| $C_{n_\beta}$ | 偏航稳定性导数 | $> 0$ | 风标稳定性 |
| $C_{m_q}$ | 俯仰阻尼 | $< 0$ | 俯仰运动阻尼 |
| $C_{l_p}$ | 滚转阻尼 | $< 0$ | 滚转运动阻尼 |
| $C_{n_r}$ | 偏航阻尼 | $< 0$ | 偏航运动阻尼 |

### 9.2 控制导数（Control Derivatives）

**主控制导数**：
| 系数 | 控制面 | 产生效果 |
|------|--------|----------|
| $C_{m_{\delta_e}}$ | 升降舵 | 俯仰力矩 |
| $C_{l_{\delta_a}}$ | 副翼 | 滚转力矩 |
| $C_{n_{\delta_r}}$ | 方向舵 | 偏航力矩 |

**交叉控制导数**：
| 系数 | 控制面 | 产生效果 |
|------|--------|----------|
| $C_{l_{\delta_r}}$ | 方向舵 | 滚转力矩（交叉） |
| $C_{n_{\delta_a}}$ | 副翼 | 偏航力矩（交叉） |

### 9.3 导数的数学定义

$$
C_{L_\alpha} \triangleq \frac{\partial C_L}{\partial \alpha}, \quad
C_{L_q} \triangleq \frac{\partial C_L}{\partial \frac{qc}{2V_a}}, \quad
C_{L_{\delta_e}} \triangleq \frac{\partial C_L}{\partial \delta_e}
$$

---

## 10. 风速模型

### 10.1 机体坐标系下的风速

风速由稳定风和干扰风（阵风）组成：

$$
\mathbf{V}_w^b = \begin{pmatrix} u_w \\ v_w \\ w_w \end{pmatrix} =
\mathcal{R}_v^b(\phi, \theta, \psi) \begin{pmatrix} w_{n_s} \\ w_{e_s} \\ w_{d_s} \end{pmatrix} +
\begin{pmatrix} u_{w_g} \\ v_{w_g} \\ w_{w_g} \end{pmatrix}
$$

其中：
- $\mathcal{R}_v^b$ = 从惯性坐标系到机体坐标系的旋转矩阵
- $(w_{n_s}, w_{e_s}, w_{d_s})^T$ = 惯性坐标系下的稳定风
- $(u_{w_g}, v_{w_g}, w_{w_g})^T$ = 机体坐标系下的干扰风（阵风）

### 10.2 机体坐标系上的空速

$$
\mathbf{V}_a^b = \begin{pmatrix} u_r \\ v_r \\ w_r \end{pmatrix} =
\begin{pmatrix} u - u_w \\ v - v_w \\ w - w_w \end{pmatrix}
$$

### 10.3 空速、攻角和侧滑角的计算

$$
V_a = \sqrt{u_r^2 + v_r^2 + w_r^2}
$$

$$
\alpha = \arctan\left(\frac{w_r}{u_r}\right)
$$

$$
\beta = \arctan\left(\frac{v_r}{\sqrt{u_r^2 + v_r^2 + w_r^2}}\right)
$$

**注意**：在实际实现中，应使用 `atan2` 函数避免除零错误。

---

## 11. 德莱登（Dryden）风速模型

### 11.1 传递函数形式

用于将白噪声转换为具有真实统计特性的干扰风：

$$
H_u(s) = \sigma_u \sqrt{\frac{2V_a}{L_u}} \cdot \frac{1}{s + \frac{V_a}{L_u}}
$$

$$
H_v(s) = \sigma_v \sqrt{\frac{3V_a}{L_v}} \cdot \frac{s + \frac{V_a}{\sqrt{3}L_v}}{\left(s + \frac{V_a}{L_v}\right)^2}
$$

$$
H_w(s) = \sigma_w \sqrt{\frac{3V_a}{L_w}} \cdot \frac{s + \frac{V_a}{\sqrt{3}L_w}}{\left(s + \frac{V_a}{L_w}\right)^2}
$$

### 11.2 参数说明

| 符号 | 含义 |
|------|------|
| $\sigma_u, \sigma_v, \sigma_w$ | 各轴干扰强度 (m/s) |
| $L_u, L_v, L_w$ | 空间波长 (m) |
| $V_a$ | 飞机空速 (m/s) |

### 11.3 德莱登风速模型参数 (表4.1)

| 干扰风描述 | 高度/m | $L_u=L_v$/m | $L_w$/m | $\sigma_u=\sigma_v$/(m/s) | $\sigma_w$/(m/s) |
|-----------|--------|-------------|---------|---------------------------|------------------|
| 高度低、轻度干扰 | 50 | 200 | 50 | 1.06 | 0.7 |
| 高度低、中度干扰 | 50 | 200 | 50 | 2.12 | 1.4 |
| 高度中等、轻度干扰 | 600 | 533 | 533 | 1.5 | 1.5 |
| 高度中等、中度干扰 | 600 | 533 | 533 | 3.0 | 3.0 |

---

## 12. 公式依赖关系图

```
输入参数
    │
    ├─► 风速模型 ─► (uw, vw, ww)
    │                    │
    │                    ▼
    │            空速计算 ─► Va, α, β
    │                    │
    ├───────────────────┘
    │
    ▼
气动系数计算
    │
    ├─► CL(α) ◄── σ(α), CLα
    │      │
    │      ▼
    ├─► CD(α) ◄── CDp, CL(α)
    │
    ├─► CX(α), CZ(α) ◄── CL(α), CD(α)
    │
    ▼
总力计算 (式4.18)
    │
    ├─► 重力分量
    ├─► 气动力 (纵向 + 横向)
    └─► 推力

总力矩计算
    │
    ├─► 滚转力矩 l
    ├─► 俯仰力矩 m
    └─► 偏航力矩 n
```

---

## 13. 与第三章的接口

第三章的运动方程需要的输入：
- 力 $(f_x, f_y, f_z)$ ─ 由本章计算
- 力矩 $(l, m, n)$ ─ 由本章计算

本章需要的状态输入（来自第三章）：
- 速度 $(u, v, w)$
- 角速度 $(p, q, r)$
- 姿态角 $(\phi, \theta, \psi)$

控制输入：
- $(\delta_e, \delta_a, \delta_r, \delta_t)$
