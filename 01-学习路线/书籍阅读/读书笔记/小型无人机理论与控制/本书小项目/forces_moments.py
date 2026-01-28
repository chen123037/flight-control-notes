"""
力与力矩计算模块
实现作用在MAV上的总力和总力矩

参考：《小型无人机理论与控制》第四章 式(4.18)和力矩方程
"""

import numpy as np

from mav_params import g
from aerodynamics import AerodynamicsModel, compute_CL, compute_CD, stability_to_body


class ForcesAndMoments:
    """力与力矩计算类"""

    def __init__(self, params):
        """
        初始化力与力矩计算器

        参数:
            params: 参数类（ZagiParams或UAVParams）
        """
        self.params = params
        self.aero = AerodynamicsModel(params)

        # 缓存常用参数
        self.m = params.m
        self.rho = params.rho
        self.S = params.S
        self.b = params.b
        self.c = params.c
        self.S_prop = params.S_prop
        self.C_prop = params.C_prop
        self.k_motor = params.k_motor
        self.k_Tp = params.k_Tp
        self.k_Omega = params.k_Omega

        # 横向气动系数
        self.C_Y0 = params.C_Y0
        self.C_Y_beta = params.C_Y_beta
        self.C_Y_p = params.C_Y_p
        self.C_Y_r = params.C_Y_r
        self.C_Y_delta_a = params.C_Y_delta_a
        # 方向舵系数（Zagi飞翼可能没有）
        self.C_Y_delta_r = getattr(params, 'C_Y_delta_r', 0)

        # 滚转力矩系数
        self.C_l0 = params.C_l0
        self.C_l_beta = params.C_l_beta
        self.C_l_p = params.C_l_p
        self.C_l_r = params.C_l_r
        self.C_l_delta_a = params.C_l_delta_a
        self.C_l_delta_r = getattr(params, 'C_l_delta_r', 0)

        # 俯仰力矩系数
        self.C_m0 = params.C_m0
        self.C_m_alpha = params.C_m_alpha
        self.C_m_q = params.C_m_q
        self.C_m_delta_e = params.C_m_delta_e

        # 偏航力矩系数
        self.C_n0 = params.C_n0
        self.C_n_beta = params.C_n_beta
        self.C_n_p = params.C_n_p
        self.C_n_r = params.C_n_r
        self.C_n_delta_a = params.C_n_delta_a
        self.C_n_delta_r = getattr(params, 'C_n_delta_r', 0)

    def compute_airspeed_angles(self, u_r, v_r, w_r):
        """
        计算空速、攻角和侧滑角

        参数:
            u_r, v_r, w_r: 相对空速在机体坐标系的分量 (m/s)

        返回:
            Va: 空速 (m/s)
            alpha: 攻角 (rad)
            beta: 侧滑角 (rad)
        """
        # 空速
        Va = np.sqrt(u_r**2 + v_r**2 + w_r**2)

        # 避免除零
        if Va < 0.1:
            Va = 0.1

        # 攻角
        if abs(u_r) < 1e-6:
            alpha = np.sign(w_r) * np.pi / 2 if w_r != 0 else 0
        else:
            alpha = np.arctan2(w_r, u_r)

        # 侧滑角
        beta = np.arcsin(np.clip(v_r / Va, -1, 1))

        return Va, alpha, beta

    def compute_gravity_force(self, phi, theta):
        """
        计算重力在机体坐标系的分量

        参数:
            phi: 滚转角 (rad)
            theta: 俯仰角 (rad)

        返回:
            fg: 重力分量 [fx_g, fy_g, fz_g]
        """
        fx_g = -self.m * g * np.sin(theta)
        fy_g = self.m * g * np.cos(theta) * np.sin(phi)
        fz_g = self.m * g * np.cos(theta) * np.cos(phi)

        return np.array([fx_g, fy_g, fz_g])

    def compute_aerodynamic_force(self, Va, alpha, beta, p, q, r, delta_e, delta_a, delta_r):
        """
        计算气动力 - 式(4.18)

        参数:
            Va: 空速 (m/s)
            alpha: 攻角 (rad)
            beta: 侧滑角 (rad)
            p, q, r: 角速度 (rad/s)
            delta_e: 升降舵偏角 (rad)
            delta_a: 副翼偏角 (rad)
            delta_r: 方向舵偏角 (rad)

        返回:
            fa: 气动力 [fx_a, fy_a, fz_a]
        """
        # 动压
        q_bar = 0.5 * self.rho * Va**2

        # 获取纵向气动系数
        coeff = self.aero.get_longitudinal_coefficients(alpha)
        C_X = coeff['C_X']
        C_X_q = coeff['C_X_q']
        C_X_delta_e = coeff['C_X_delta_e']
        C_Z = coeff['C_Z']
        C_Z_q = coeff['C_Z_q']
        C_Z_delta_e = coeff['C_Z_delta_e']

        # 纵向气动力 (X和Z方向)
        # 无纲量俯仰角速度
        q_hat = q * self.c / (2 * Va) if Va > 0.1 else 0

        fx_a = q_bar * self.S * (C_X + C_X_q * q_hat + C_X_delta_e * delta_e)
        fz_a = q_bar * self.S * (C_Z + C_Z_q * q_hat + C_Z_delta_e * delta_e)

        # 横向气动力 (Y方向) - 式(4.14)
        # 无纲量滚转和偏航角速度
        p_hat = p * self.b / (2 * Va) if Va > 0.1 else 0
        r_hat = r * self.b / (2 * Va) if Va > 0.1 else 0

        fy_a = q_bar * self.S * (
            self.C_Y0 +
            self.C_Y_beta * beta +
            self.C_Y_p * p_hat +
            self.C_Y_r * r_hat +
            self.C_Y_delta_a * delta_a +
            self.C_Y_delta_r * delta_r
        )

        return np.array([fx_a, fy_a, fz_a])

    def compute_propeller_force(self, Va, delta_t):
        """
        计算螺旋桨推力

        参数:
            Va: 空速 (m/s)
            delta_t: 油门设置 (0~1)

        返回:
            fp: 推力 [fx_p, fy_p, fz_p]
        """
        # 推力模型：0.5 * ρ * S_prop * C_prop * [(k_motor * δt)² - Va²]
        thrust = 0.5 * self.rho * self.S_prop * self.C_prop * (
            (self.k_motor * delta_t)**2 - Va**2
        )

        # 推力沿机体X轴
        return np.array([thrust, 0, 0])

    def compute_aerodynamic_moment(self, Va, alpha, beta, p, q, r, delta_e, delta_a, delta_r):
        """
        计算气动力矩

        参数:
            Va: 空速 (m/s)
            alpha: 攻角 (rad)
            beta: 侧滑角 (rad)
            p, q, r: 角速度 (rad/s)
            delta_e: 升降舵偏角 (rad)
            delta_a: 副翼偏角 (rad)
            delta_r: 方向舵偏角 (rad)

        返回:
            ma: 气动力矩 [l, m, n]
        """
        # 动压
        q_bar = 0.5 * self.rho * Va**2

        # 无纲量角速度
        p_hat = p * self.b / (2 * Va) if Va > 0.1 else 0
        q_hat = q * self.c / (2 * Va) if Va > 0.1 else 0
        r_hat = r * self.b / (2 * Va) if Va > 0.1 else 0

        # 滚转力矩 l - 式(4.15)
        l = q_bar * self.S * self.b * (
            self.C_l0 +
            self.C_l_beta * beta +
            self.C_l_p * p_hat +
            self.C_l_r * r_hat +
            self.C_l_delta_a * delta_a +
            self.C_l_delta_r * delta_r
        )

        # 俯仰力矩 m
        m = q_bar * self.S * self.c * (
            self.C_m0 +
            self.C_m_alpha * alpha +
            self.C_m_q * q_hat +
            self.C_m_delta_e * delta_e
        )

        # 偏航力矩 n - 式(4.16)
        n = q_bar * self.S * self.b * (
            self.C_n0 +
            self.C_n_beta * beta +
            self.C_n_p * p_hat +
            self.C_n_r * r_hat +
            self.C_n_delta_a * delta_a +
            self.C_n_delta_r * delta_r
        )

        return np.array([l, m, n])

    def compute_propeller_moment(self, delta_t):
        """
        计算螺旋桨扭矩

        参数:
            delta_t: 油门设置 (0~1)

        返回:
            mp: 螺旋桨扭矩 [l_p, m_p, n_p]
        """
        # 螺旋桨扭矩：-k_Tp * (k_Omega * δt)²
        l_p = -self.k_Tp * (self.k_Omega * delta_t)**2

        return np.array([l_p, 0, 0])

    def compute(self, state, wind, controls):
        """
        计算总力和总力矩

        参数:
            state: 状态向量 [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
            wind: 风速 [u_w, v_w, w_w] 在机体坐标系
            controls: 控制输入 [delta_e, delta_a, delta_r, delta_t]

        返回:
            forces: 总力 [fx, fy, fz]
            moments: 总力矩 [l, m, n]
            Va: 空速
            alpha: 攻角
            beta: 侧滑角
        """
        # 解包状态
        u, v, w = state[3:6]
        phi, theta = state[6], state[7]
        p, q, r = state[9:12]

        # 解包控制
        delta_e, delta_a, delta_r, delta_t = controls

        # 计算相对空速
        u_w, v_w, w_w = wind
        u_r = u - u_w
        v_r = v - v_w
        w_r = w - w_w

        # 计算空速、攻角、侧滑角
        Va, alpha, beta = self.compute_airspeed_angles(u_r, v_r, w_r)

        # 计算各项力
        fg = self.compute_gravity_force(phi, theta)
        fa = self.compute_aerodynamic_force(Va, alpha, beta, p, q, r,
                                            delta_e, delta_a, delta_r)
        fp = self.compute_propeller_force(Va, delta_t)

        # 总力
        forces = fg + fa + fp

        # 计算各项力矩
        ma = self.compute_aerodynamic_moment(Va, alpha, beta, p, q, r,
                                             delta_e, delta_a, delta_r)
        mp = self.compute_propeller_moment(delta_t)

        # 总力矩
        moments = ma + mp

        return forces, moments, Va, alpha, beta


def compute_trim_throttle(params, Va_trim, gamma=0):
    """
    估算平飞所需油门（简化计算）

    参数:
        params: 飞机参数
        Va_trim: 目标空速 (m/s)
        gamma: 航迹角 (rad)

    返回:
        delta_t: 估算油门值
    """
    # 简化估算：推力等于阻力
    # D ≈ 0.5 * ρ * Va² * S * CD0
    D = 0.5 * params.rho * Va_trim**2 * params.S * params.C_D_p

    # 加上爬升分量
    D += params.m * g * np.sin(gamma)

    # 推力 = 0.5 * ρ * S_prop * C_prop * (k_motor * δt)²
    # 忽略 -Va² 项的简化
    if params.k_motor > 0:
        thrust_coeff = 0.5 * params.rho * params.S_prop * params.C_prop * params.k_motor**2
        delta_t = np.sqrt(D / thrust_coeff)
        delta_t = np.clip(delta_t, 0, 1)
    else:
        delta_t = 0.5

    return delta_t


if __name__ == "__main__":
    # 测试力与力矩计算
    from mav_params import ZagiParams

    print("=== 力与力矩计算测试 ===")

    # 创建计算器
    fm = ForcesAndMoments(ZagiParams)

    # 测试状态：平飞
    Va = 20.0  # m/s
    state = np.zeros(12)
    state[3] = Va  # u = Va
    state[7] = np.deg2rad(2)  # 小俯仰角

    wind = np.zeros(3)
    delta_t = compute_trim_throttle(ZagiParams, Va)
    controls = [0, 0, 0, delta_t]  # [delta_e, delta_a, delta_r, delta_t]

    print(f"\n估算平飞油门: {delta_t:.4f}")

    # 计算力与力矩
    forces, moments, Va_calc, alpha, beta = fm.compute(state, wind, controls)

    print(f"\n空速 Va = {Va_calc:.2f} m/s")
    print(f"攻角 α = {np.rad2deg(alpha):.2f}°")
    print(f"侧滑角 β = {np.rad2deg(beta):.2f}°")

    print(f"\n总力 (N):")
    print(f"  fx = {forces[0]:.4f}")
    print(f"  fy = {forces[1]:.4f}")
    print(f"  fz = {forces[2]:.4f}")

    print(f"\n总力矩 (N·m):")
    print(f"  l = {moments[0]:.6f}")
    print(f"  m = {moments[1]:.6f}")
    print(f"  n = {moments[2]:.6f}")

    # 分解各项力
    print("\n=== 力分解 ===")
    fg = fm.compute_gravity_force(state[6], state[7])
    print(f"重力: [{fg[0]:.4f}, {fg[1]:.4f}, {fg[2]:.4f}]")

    fa = fm.compute_aerodynamic_force(Va_calc, alpha, beta,
                                       state[9], state[10], state[11],
                                       controls[0], controls[1], controls[2])
    print(f"气动力: [{fa[0]:.4f}, {fa[1]:.4f}, {fa[2]:.4f}]")

    fp = fm.compute_propeller_force(Va_calc, controls[3])
    print(f"推力: [{fp[0]:.4f}, {fp[1]:.4f}, {fp[2]:.4f}]")
