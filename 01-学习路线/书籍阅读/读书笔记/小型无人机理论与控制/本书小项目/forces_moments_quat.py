"""
基于四元数的力与力矩计算模块
实现作用在MAV上的总力和总力矩

支持四元数和欧拉角两种输入方式
参考：《小型无人机理论与控制》附录B 式(B.8)~(B.17)
"""

import numpy as np

from mav_params import g
from aerodynamics import AerodynamicsModel, compute_CL, compute_CD
from quaternion import (
    quaternion_to_euler,
    euler_to_quaternion,
    gravity_body_frame,
    quaternion_to_rotation_matrix
)
from mav_dynamics_quat import StateIndexQuat


class ForcesAndMomentsQuat:
    """
    基于四元数的力与力矩计算类

    支持13维四元数状态或12维欧拉角状态
    """

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
        计算空速、攻角和侧滑角 - 式(2.8)

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

    def compute_gravity_force_quat(self, quaternion):
        """
        使用四元数计算重力在机体坐标系的分量

        参数:
            quaternion: 四元数 [e0, e1, e2, e3]

        返回:
            fg: 重力分量 [fx_g, fy_g, fz_g]
        """
        return gravity_body_frame(quaternion, self.m, g)

    def compute_gravity_force_euler(self, phi, theta):
        """
        使用欧拉角计算重力在机体坐标系的分量（传统方法）

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
        计算气动力

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

        # 无纲量角速度
        q_hat = q * self.c / (2 * Va) if Va > 0.1 else 0
        p_hat = p * self.b / (2 * Va) if Va > 0.1 else 0
        r_hat = r * self.b / (2 * Va) if Va > 0.1 else 0

        # 纵向气动力 (X和Z方向)
        fx_a = q_bar * self.S * (C_X + C_X_q * q_hat + C_X_delta_e * delta_e)
        fz_a = q_bar * self.S * (C_Z + C_Z_q * q_hat + C_Z_delta_e * delta_e)

        # 横向气动力 (Y方向)
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
        thrust = 0.5 * self.rho * self.S_prop * self.C_prop * (
            (self.k_motor * delta_t)**2 - Va**2
        )

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

        # 滚转力矩 l
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

        # 偏航力矩 n
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
        l_p = -self.k_Tp * (self.k_Omega * delta_t)**2

        return np.array([l_p, 0, 0])

    def compute(self, state, wind, controls, use_quaternion=True):
        """
        计算总力和总力矩

        参数:
            state: 状态向量
                - 13维: 四元数形式 [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]
                - 12维: 欧拉角形式 [pn,pe,pd,u,v,w,φ,θ,ψ,p,q,r]
            wind: 风速 [u_w, v_w, w_w] 在机体坐标系
            controls: 控制输入 [delta_e, delta_a, delta_r, delta_t]
            use_quaternion: 是否使用四元数计算重力

        返回:
            forces: 总力 [fx, fy, fz]
            moments: 总力矩 [l, m, n]
            Va: 空速
            alpha: 攻角
            beta: 侧滑角
        """
        state = np.array(state)

        # 根据状态维度解包
        if len(state) == 13:
            # 四元数形式
            u, v, w = state[3:6]
            quaternion = state[6:10]
            p, q, r = state[10:13]

            if use_quaternion:
                fg = self.compute_gravity_force_quat(quaternion)
            else:
                phi, theta, psi = quaternion_to_euler(quaternion)
                fg = self.compute_gravity_force_euler(phi, theta)

        elif len(state) == 12:
            # 欧拉角形式
            u, v, w = state[3:6]
            phi, theta = state[6], state[7]
            p, q, r = state[9:12]

            if use_quaternion:
                quaternion = euler_to_quaternion(phi, theta, state[8])
                fg = self.compute_gravity_force_quat(quaternion)
            else:
                fg = self.compute_gravity_force_euler(phi, theta)
        else:
            raise ValueError(f"状态向量长度必须是12或13，当前为{len(state)}")

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
    D = 0.5 * params.rho * Va_trim**2 * params.S * params.C_D_p

    # 加上爬升分量
    D += params.m * g * np.sin(gamma)

    # 推力系数
    if params.k_motor > 0:
        thrust_coeff = 0.5 * params.rho * params.S_prop * params.C_prop * params.k_motor**2
        delta_t = np.sqrt(D / thrust_coeff)
        delta_t = np.clip(delta_t, 0, 1)
    else:
        delta_t = 0.5

    return delta_t


# ==================== 测试 ====================

if __name__ == "__main__":
    from mav_params import ZagiParams

    print("=" * 50)
    print("四元数力与力矩计算测试")
    print("=" * 50)

    # 创建计算器
    fm = ForcesAndMomentsQuat(ZagiParams)

    # 测试状态
    Va = 20.0

    # 欧拉角状态 (12维)
    state_euler = np.zeros(12)
    state_euler[3] = Va  # u = Va
    state_euler[7] = np.deg2rad(5)  # θ = 5°

    # 四元数状态 (13维)
    state_quat = np.zeros(13)
    state_quat[3] = Va
    e = euler_to_quaternion(0, np.deg2rad(5), 0)
    state_quat[6:10] = e

    wind = np.zeros(3)
    delta_t = compute_trim_throttle(ZagiParams, Va)
    controls = [0, 0, 0, delta_t]

    print(f"\n估算平飞油门: {delta_t:.4f}")

    # 使用欧拉角状态计算
    print("\n--- 欧拉角状态输入 ---")
    forces_e, moments_e, Va_e, alpha_e, beta_e = fm.compute(state_euler, wind, controls, use_quaternion=False)
    print(f"空速: {Va_e:.2f} m/s, 攻角: {np.rad2deg(alpha_e):.2f}°")
    print(f"总力: [{forces_e[0]:.4f}, {forces_e[1]:.4f}, {forces_e[2]:.4f}]")

    # 使用四元数状态计算
    print("\n--- 四元数状态输入 ---")
    forces_q, moments_q, Va_q, alpha_q, beta_q = fm.compute(state_quat, wind, controls, use_quaternion=True)
    print(f"空速: {Va_q:.2f} m/s, 攻角: {np.rad2deg(alpha_q):.2f}°")
    print(f"总力: [{forces_q[0]:.4f}, {forces_q[1]:.4f}, {forces_q[2]:.4f}]")

    # 对比
    print("\n--- 对比 ---")
    print(f"力差异: {np.linalg.norm(forces_e - forces_q):.2e}")
    print(f"力矩差异: {np.linalg.norm(moments_e - moments_q):.2e}")

    print("\n测试完成!")
