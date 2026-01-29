"""
基于四元数的MAV动力学模块
实现12状态（13维向量）6自由度动力学方程

内部使用四元数进行姿态计算，避免万向锁问题
对外提供欧拉角接口，保持兼容性

参考：《小型无人机理论与控制》附录B
"""

import numpy as np
from quaternion import (
    euler_to_quaternion,
    quaternion_to_euler,
    quaternion_derivative,
    quaternion_to_rotation_matrix,
    normalize_quaternion,
    gravity_body_frame,
    quaternion_norm_error
)
from mav_params import compute_gamma_coefficients


class StateIndexQuat:
    """四元数状态向量索引（13维）"""
    PN = 0      # 北向位置
    PE = 1      # 东向位置
    PD = 2      # 向下位置（高度 = -PD）
    U = 3       # 机体x轴速度
    V = 4       # 机体y轴速度
    W = 5       # 机体z轴速度
    E0 = 6      # 四元数 e0 (标量部分)
    E1 = 7      # 四元数 e1
    E2 = 8      # 四元数 e2
    E3 = 9      # 四元数 e3
    P = 10      # 滚转角速度
    Q = 11      # 俯仰角速度
    R = 12      # 偏航角速度


class MAVDynamicsQuat:
    """
    基于四元数的MAV动力学类

    状态向量 (13维):
        [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]

    特性:
        - 内部使用四元数，无万向锁
        - 自动四元数归一化
        - 兼容欧拉角输入/输出
    """

    def __init__(self, mass, Jx, Jy, Jz, Jxz, lambda_quat=1000.0):
        """
        初始化动力学模型

        参数:
            mass: 质量 (kg)
            Jx, Jy, Jz: 转动惯量 (kg·m²)
            Jxz: 惯性积 (kg·m²)
            lambda_quat: 四元数正交控制增益（建议1000）
        """
        self.mass = mass
        self.Jx = Jx
        self.Jy = Jy
        self.Jz = Jz
        self.Jxz = Jxz
        self.lambda_quat = lambda_quat

        # 计算Gamma系数
        self.Gamma = compute_gamma_coefficients(Jx, Jy, Jz, Jxz)

        # 状态向量（13维）
        self._state = np.zeros(13)
        self._state[StateIndexQuat.E0] = 1.0  # 初始四元数为单位四元数

    def set_state(self, state):
        """
        设置状态

        参数:
            state: 状态向量
                - 如果是13维: 直接使用（四元数形式）
                - 如果是12维: 假定为欧拉角形式，自动转换
        """
        state = np.array(state, dtype=float)

        if len(state) == 13:
            # 四元数形式
            self._state = state.copy()
        elif len(state) == 12:
            # 欧拉角形式，转换为四元数
            self._state = np.zeros(13)
            self._state[0:6] = state[0:6]  # 位置和速度
            phi, theta, psi = state[6:9]
            self._state[6:10] = euler_to_quaternion(phi, theta, psi)
            self._state[10:13] = state[9:12]  # 角速度
        else:
            raise ValueError(f"状态向量长度必须是12或13，当前为{len(state)}")

        # 归一化四元数
        self._state[6:10] = normalize_quaternion(self._state[6:10])

    def get_state(self):
        """获取状态向量（13维四元数形式）"""
        return self._state.copy()

    def get_state_euler(self):
        """获取状态向量（12维欧拉角形式）"""
        state_euler = np.zeros(12)
        state_euler[0:6] = self._state[0:6]  # 位置和速度
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        state_euler[6:9] = [phi, theta, psi]
        state_euler[9:12] = self._state[10:13]  # 角速度
        return state_euler

    def get_quaternion(self):
        """获取四元数 [e0, e1, e2, e3]"""
        return self._state[6:10].copy()

    def get_euler(self):
        """获取欧拉角 (phi, theta, psi)"""
        return quaternion_to_euler(self._state[6:10])

    def get_rotation_matrix(self):
        """获取旋转矩阵 R_b^i（机体系到惯性系）"""
        return quaternion_to_rotation_matrix(self._state[6:10])

    def quaternion_norm_error(self):
        """获取四元数范数误差"""
        return quaternion_norm_error(self._state[6:10])

    def equations_of_motion(self, t, state, forces, moments):
        """
        运动方程 - 式(B.5)~(B.17)

        参数:
            t: 时间
            state: 状态向量 (13维)
            forces: 机体坐标系下的总力 [fx, fy, fz]
            moments: 机体坐标系下的总力矩 [l, m, n]

        返回:
            state_dot: 状态导数 (13维)
        """
        # 解包状态
        pn, pe, pd = state[0:3]
        u, v, w = state[3:6]
        e0, e1, e2, e3 = state[6:10]
        p, q, r = state[10:13]

        # 解包力和力矩
        fx, fy, fz = forces
        l, m_moment, n = moments  # m_moment 避免与质量m混淆

        # 解包Gamma系数
        G = self.Gamma
        G1 = G['Gamma1']
        G2 = G['Gamma2']
        G3 = G['Gamma3']
        G4 = G['Gamma4']
        G5 = G['Gamma5']
        G6 = G['Gamma6']
        G7 = G['Gamma7']
        G8 = G['Gamma8']

        # ==================== 位置微分 - 式(B.5)~(B.7) ====================
        # 旋转矩阵
        R = quaternion_to_rotation_matrix([e0, e1, e2, e3])
        vel_body = np.array([u, v, w])
        vel_inertial = R @ vel_body

        pn_dot = vel_inertial[0]
        pe_dot = vel_inertial[1]
        pd_dot = vel_inertial[2]

        # ==================== 速度微分 - 式(B.2) ====================
        u_dot = r * v - q * w + fx / self.mass
        v_dot = p * w - r * u + fy / self.mass
        w_dot = q * u - p * v + fz / self.mass

        # ==================== 四元数微分 - 式(B.3) with 正交控制 ====================
        e_dot = quaternion_derivative([e0, e1, e2, e3], p, q, r, self.lambda_quat)
        e0_dot, e1_dot, e2_dot, e3_dot = e_dot

        # ==================== 角速度微分 - 式(B.4) ====================
        p_dot = G1 * p * q - G2 * q * r + G3 * l + G4 * n
        q_dot = G5 * p * r - G6 * (p**2 - r**2) + m_moment / self.Jy
        r_dot = G7 * p * q - G1 * q * r + G4 * l + G8 * n

        # 组装状态导数
        state_dot = np.array([
            pn_dot, pe_dot, pd_dot,
            u_dot, v_dot, w_dot,
            e0_dot, e1_dot, e2_dot, e3_dot,
            p_dot, q_dot, r_dot
        ])

        return state_dot

    def rk4_step(self, t, forces, moments, dt):
        """
        RK4积分一步

        参数:
            t: 当前时间
            forces: 机体坐标系下的总力 [fx, fy, fz]
            moments: 机体坐标系下的总力矩 [l, m, n]
            dt: 时间步长

        返回:
            更新后的状态
        """
        state = self._state

        k1 = self.equations_of_motion(t, state, forces, moments)
        k2 = self.equations_of_motion(t + dt/2, state + dt/2 * k1, forces, moments)
        k3 = self.equations_of_motion(t + dt/2, state + dt/2 * k2, forces, moments)
        k4 = self.equations_of_motion(t + dt, state + dt * k3, forces, moments)

        self._state = state + dt / 6 * (k1 + 2*k2 + 2*k3 + k4)

        # 归一化四元数
        self._state[6:10] = normalize_quaternion(self._state[6:10])

        return self._state.copy()


class MAVDynamicsQuatFull:
    """
    完整的四元数动力学模型（含气动力计算）

    整合了动力学方程和气动力计算，实现式(B.5)~(B.17)的完整形式
    """

    def __init__(self, params, lambda_quat=1000.0):
        """
        初始化

        参数:
            params: 飞机参数类（ZagiParams或UAVParams）
            lambda_quat: 四元数正交控制增益
        """
        self.params = params
        self.lambda_quat = lambda_quat

        # 质量和惯性
        self.m = params.m
        self.Jx = params.Jx
        self.Jy = params.Jy
        self.Jz = params.Jz
        self.Jxz = params.Jxz

        # 气动参数
        self.rho = params.rho
        self.S = params.S
        self.b = params.b
        self.c = params.c
        self.S_prop = params.S_prop
        self.C_prop = params.C_prop
        self.k_motor = params.k_motor

        # 计算Gamma系数
        self.Gamma = compute_gamma_coefficients(self.Jx, self.Jy, self.Jz, self.Jxz)

        # 计算C_p和C_r系数
        self._compute_Cp_Cr_coefficients()

        # 状态向量
        self._state = np.zeros(13)
        self._state[StateIndexQuat.E0] = 1.0

    def _compute_Cp_Cr_coefficients(self):
        """计算组合的滚转和偏航力矩系数"""
        p = self.params
        G = self.Gamma
        G3, G4, G8 = G['Gamma3'], G['Gamma4'], G['Gamma8']

        # C_p 系列（滚转）
        self.C_p0 = G3 * p.C_l0 + G4 * p.C_n0
        self.C_p_beta = G3 * p.C_l_beta + G4 * p.C_n_beta
        self.C_p_p = G3 * p.C_l_p + G4 * p.C_n_p
        self.C_p_r = G3 * p.C_l_r + G4 * p.C_n_r
        self.C_p_delta_a = G3 * p.C_l_delta_a + G4 * p.C_n_delta_a
        self.C_p_delta_r = G3 * getattr(p, 'C_l_delta_r', 0) + G4 * getattr(p, 'C_n_delta_r', 0)

        # C_r 系列（偏航）
        self.C_r0 = G4 * p.C_l0 + G8 * p.C_n0
        self.C_r_beta = G4 * p.C_l_beta + G8 * p.C_n_beta
        self.C_r_p = G4 * p.C_l_p + G8 * p.C_n_p
        self.C_r_r = G4 * p.C_l_r + G8 * p.C_n_r
        self.C_r_delta_a = G4 * p.C_l_delta_a + G8 * p.C_n_delta_a
        self.C_r_delta_r = G4 * getattr(p, 'C_l_delta_r', 0) + G8 * getattr(p, 'C_n_delta_r', 0)

    def set_state(self, state):
        """设置状态（支持12维欧拉角或13维四元数）"""
        state = np.array(state, dtype=float)

        if len(state) == 13:
            self._state = state.copy()
        elif len(state) == 12:
            self._state = np.zeros(13)
            self._state[0:6] = state[0:6]
            phi, theta, psi = state[6:9]
            self._state[6:10] = euler_to_quaternion(phi, theta, psi)
            self._state[10:13] = state[9:12]
        else:
            raise ValueError(f"状态向量长度必须是12或13，当前为{len(state)}")

        self._state[6:10] = normalize_quaternion(self._state[6:10])

    def get_state(self):
        """获取状态（13维四元数形式）"""
        return self._state.copy()

    def get_state_euler(self):
        """获取状态（12维欧拉角形式）"""
        state_euler = np.zeros(12)
        state_euler[0:6] = self._state[0:6]
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        state_euler[6:9] = [phi, theta, psi]
        state_euler[9:12] = self._state[10:13]
        return state_euler

    def get_euler(self):
        """获取欧拉角"""
        return quaternion_to_euler(self._state[6:10])

    def get_quaternion(self):
        """获取四元数"""
        return self._state[6:10].copy()


# ==================== 辅助函数 ====================

def state_euler_to_quat(state_euler):
    """
    将欧拉角状态向量转换为四元数状态向量

    参数:
        state_euler: 12维欧拉角状态 [pn,pe,pd,u,v,w,φ,θ,ψ,p,q,r]

    返回:
        13维四元数状态 [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]
    """
    state_quat = np.zeros(13)
    state_quat[0:6] = state_euler[0:6]
    phi, theta, psi = state_euler[6:9]
    state_quat[6:10] = euler_to_quaternion(phi, theta, psi)
    state_quat[10:13] = state_euler[9:12]
    return state_quat


def state_quat_to_euler(state_quat):
    """
    将四元数状态向量转换为欧拉角状态向量

    参数:
        state_quat: 13维四元数状态

    返回:
        12维欧拉角状态
    """
    state_euler = np.zeros(12)
    state_euler[0:6] = state_quat[0:6]
    phi, theta, psi = quaternion_to_euler(state_quat[6:10])
    state_euler[6:9] = [phi, theta, psi]
    state_euler[9:12] = state_quat[10:13]
    return state_euler


# ==================== 测试 ====================

if __name__ == "__main__":
    from mav_params import ZagiParams

    print("=" * 50)
    print("四元数动力学模块测试")
    print("=" * 50)

    # 创建动力学模型
    dynamics = MAVDynamicsQuat(
        mass=ZagiParams.m,
        Jx=ZagiParams.Jx,
        Jy=ZagiParams.Jy,
        Jz=ZagiParams.Jz,
        Jxz=ZagiParams.Jxz
    )

    # 测试1: 状态设置（欧拉角输入）
    print("\n--- 测试1: 欧拉角输入 ---")
    initial_euler = np.zeros(12)
    initial_euler[3] = 25.0  # u = 25 m/s
    initial_euler[6] = np.deg2rad(10)  # φ = 10°
    initial_euler[7] = np.deg2rad(5)   # θ = 5°
    initial_euler[8] = np.deg2rad(30)  # ψ = 30°

    dynamics.set_state(initial_euler)
    print(f"输入欧拉角: φ=10°, θ=5°, ψ=30°")

    phi, theta, psi = dynamics.get_euler()
    print(f"输出欧拉角: φ={np.rad2deg(phi):.1f}°, θ={np.rad2deg(theta):.1f}°, ψ={np.rad2deg(psi):.1f}°")

    e = dynamics.get_quaternion()
    print(f"四元数: [{e[0]:.4f}, {e[1]:.4f}, {e[2]:.4f}, {e[3]:.4f}]")
    print(f"四元数范数误差: {dynamics.quaternion_norm_error():.2e}")

    # 测试2: 简单积分
    print("\n--- 测试2: 简单积分（无外力）---")
    dynamics.set_state(initial_euler)

    # 零力和零力矩
    forces = np.array([0, 0, dynamics.mass * 9.81])  # 只有重力
    moments = np.array([0, 0, 0])

    print("积分10步（dt=0.01）...")
    for i in range(10):
        dynamics.rk4_step(i * 0.01, forces, moments, 0.01)

    print(f"四元数范数误差: {dynamics.quaternion_norm_error():.2e}")
    phi, theta, psi = dynamics.get_euler()
    print(f"最终欧拉角: φ={np.rad2deg(phi):.2f}°, θ={np.rad2deg(theta):.2f}°, ψ={np.rad2deg(psi):.2f}°")

    # 测试3: Gamma系数
    print("\n--- 测试3: Gamma系数 ---")
    for key, val in dynamics.Gamma.items():
        print(f"  {key} = {val:.6f}")

    print("\n测试完成!")
