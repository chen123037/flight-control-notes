"""
MAV六自由度动力学模块
实现式(3.14)~(3.17)的运动方程

状态向量 x = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
         位置(NED)    速度(机体系)   欧拉角        角速度(机体系)
"""

import numpy as np
from scipy.integrate import solve_ivp

from mav_params import compute_gamma_coefficients, g


class MAVDynamics:
    """MAV六自由度动力学模型"""

    def __init__(self, mass, Jx, Jy, Jz, Jxz, initial_state=None):
        """
        初始化MAV动力学模型

        参数:
            mass: 质量 (kg)
            Jx, Jy, Jz: 转动惯量 (kg·m²)
            Jxz: 惯性积 (kg·m²)
            initial_state: 初始状态向量 (12,)
        """
        self.mass = mass
        self.Jx = Jx
        self.Jy = Jy
        self.Jz = Jz
        self.Jxz = Jxz

        # 计算Gamma系数
        gamma = compute_gamma_coefficients(Jx, Jy, Jz, Jxz)
        self.Gamma = gamma["Gamma"]
        self.Gamma1 = gamma["Gamma1"]
        self.Gamma2 = gamma["Gamma2"]
        self.Gamma3 = gamma["Gamma3"]
        self.Gamma4 = gamma["Gamma4"]
        self.Gamma5 = gamma["Gamma5"]
        self.Gamma6 = gamma["Gamma6"]
        self.Gamma7 = gamma["Gamma7"]
        self.Gamma8 = gamma["Gamma8"]

        # 初始状态
        if initial_state is None:
            self.state = np.zeros(12)
        else:
            self.state = np.array(initial_state, dtype=float)

        # 存储仿真历史
        self.time_history = []
        self.state_history = []

    def set_state(self, state):
        """设置当前状态"""
        self.state = np.array(state, dtype=float)

    def get_state(self):
        """获取当前状态"""
        return self.state.copy()

    def rotation_matrix_body_to_inertial(self, phi, theta, psi):
        """
        计算从机体坐标系到惯性坐标系的旋转矩阵 R_b^v

        参数:
            phi: 滚转角 (rad)
            theta: 俯仰角 (rad)
            psi: 偏航角 (rad)

        返回:
            3x3旋转矩阵
        """
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)

        R = np.array([
            [c_theta * c_psi,
             s_phi * s_theta * c_psi - c_phi * s_psi,
             c_phi * s_theta * c_psi + s_phi * s_psi],
            [c_theta * s_psi,
             s_phi * s_theta * s_psi + c_phi * c_psi,
             c_phi * s_theta * s_psi - s_phi * c_psi],
            [-s_theta,
             s_phi * c_theta,
             c_phi * c_theta]
        ])
        return R

    def euler_kinematics_matrix(self, phi, theta):
        """
        计算欧拉角运动学矩阵

        参数:
            phi: 滚转角 (rad)
            theta: 俯仰角 (rad)

        返回:
            3x3矩阵，将角速度[p,q,r]映射到欧拉角导数
        """
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        t_theta = np.tan(theta)

        # 避免除零（theta接近±90度时）
        if np.abs(c_theta) < 1e-10:
            c_theta = 1e-10 * np.sign(c_theta) if c_theta != 0 else 1e-10

        sec_theta = 1.0 / c_theta

        return np.array([
            [1, s_phi * t_theta, c_phi * t_theta],
            [0, c_phi, -s_phi],
            [0, s_phi * sec_theta, c_phi * sec_theta]
        ])

    def equations_of_motion(self, t, state, forces, moments):
        """
        六自由度运动方程（ODE右端函数）

        参数:
            t: 时间
            state: 状态向量 [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
            forces: 机体坐标系下的力 [fx, fy, fz]
            moments: 机体坐标系下的力矩 [l, m, n]

        返回:
            状态导数向量
        """
        # 解包状态
        pn, pe, pd = state[0:3]  # 位置
        u, v, w = state[3:6]  # 速度
        phi, theta, psi = state[6:9]  # 欧拉角
        p, q, r = state[9:12]  # 角速度

        # 解包力和力矩
        fx, fy, fz = forces
        l, m, n = moments

        # === 式(3.14) 平移运动学方程 ===
        R = self.rotation_matrix_body_to_inertial(phi, theta, psi)
        vel_body = np.array([u, v, w])
        vel_inertial = R @ vel_body

        pn_dot = vel_inertial[0]
        pe_dot = vel_inertial[1]
        pd_dot = vel_inertial[2]

        # === 式(3.15) 平移动力学方程 ===
        u_dot = r * v - q * w + fx / self.mass
        v_dot = p * w - r * u + fy / self.mass
        w_dot = q * u - p * v + fz / self.mass

        # === 式(3.16) 姿态运动学方程 ===
        E = self.euler_kinematics_matrix(phi, theta)
        omega_body = np.array([p, q, r])
        euler_dot = E @ omega_body

        phi_dot = euler_dot[0]
        theta_dot = euler_dot[1]
        psi_dot = euler_dot[2]

        # === 式(3.17) 转动动力学方程 ===
        p_dot = (self.Gamma1 * p * q - self.Gamma2 * q * r +
                 self.Gamma3 * l + self.Gamma4 * n)
        q_dot = (self.Gamma5 * p * r - self.Gamma6 * (p**2 - r**2) +
                 m / self.Jy)
        r_dot = (self.Gamma7 * p * q - self.Gamma1 * q * r +
                 self.Gamma4 * l + self.Gamma8 * n)

        return np.array([
            pn_dot, pe_dot, pd_dot,
            u_dot, v_dot, w_dot,
            phi_dot, theta_dot, psi_dot,
            p_dot, q_dot, r_dot
        ])

    def simulate(self, t_span, forces_func, moments_func, dt=0.01):
        """
        运行仿真

        参数:
            t_span: 时间范围 (t_start, t_end)
            forces_func: 力函数 forces_func(t, state) -> [fx, fy, fz]
            moments_func: 力矩函数 moments_func(t, state) -> [l, m, n]
            dt: 输出时间步长

        返回:
            t: 时间数组
            states: 状态历史 (n_times, 12)
        """
        t_eval = np.arange(t_span[0], t_span[1], dt)

        def ode_func(t, state):
            forces = forces_func(t, state)
            moments = moments_func(t, state)
            return self.equations_of_motion(t, state, forces, moments)

        sol = solve_ivp(
            ode_func,
            t_span,
            self.state,
            method="RK45",
            t_eval=t_eval,
            rtol=1e-6,
            atol=1e-9
        )

        self.time_history = sol.t
        self.state_history = sol.y.T  # (n_times, 12)

        # 更新最终状态
        self.state = sol.y[:, -1]

        return sol.t, sol.y.T

    def get_gravity_force_body(self, phi, theta):
        """
        计算机体坐标系下的重力分量

        参数:
            phi: 滚转角 (rad)
            theta: 俯仰角 (rad)

        返回:
            [fx_g, fy_g, fz_g]: 重力在机体系下的分量
        """
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)

        # 重力在惯性系下是 [0, 0, mg]（NED坐标系，向下为正）
        # 转换到机体系
        fx_g = -self.mass * g * s_theta
        fy_g = self.mass * g * c_theta * s_phi
        fz_g = self.mass * g * c_theta * c_phi

        return np.array([fx_g, fy_g, fz_g])


def create_mav_from_params(params_class, initial_state=None):
    """
    从参数类创建MAV动力学模型

    参数:
        params_class: ZagiParams 或 UAVParams 类
        initial_state: 初始状态

    返回:
        MAVDynamics实例
    """
    return MAVDynamics(
        mass=params_class.m,
        Jx=params_class.Jx,
        Jy=params_class.Jy,
        Jz=params_class.Jz,
        Jxz=params_class.Jxz,
        initial_state=initial_state
    )


# 状态索引常量，方便访问
class StateIndex:
    PN = 0  # 北向位置
    PE = 1  # 东向位置
    PD = 2  # 下向位置（高度取负）
    U = 3   # 前向速度
    V = 4   # 侧向速度
    W = 5   # 垂向速度
    PHI = 6    # 滚转角
    THETA = 7  # 俯仰角
    PSI = 8    # 偏航角
    P = 9   # 滚转角速度
    Q = 10  # 俯仰角速度
    R = 11  # 偏航角速度


if __name__ == "__main__":
    # 简单测试
    from mav_params import ZagiParams

    print("创建Zagi飞翼动力学模型...")
    mav = create_mav_from_params(ZagiParams)

    # 设置初始状态：向前飞行 20 m/s
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 20.0  # 前向速度
    initial_state[StateIndex.PD] = -100.0  # 高度100m（NED坐标系向下为正）
    mav.set_state(initial_state)

    # 定义力和力矩函数（仅重力）
    def forces(t, state):
        phi = state[StateIndex.PHI]
        theta = state[StateIndex.THETA]
        return mav.get_gravity_force_body(phi, theta)

    def moments(t, state):
        return np.array([0.0, 0.0, 0.0])

    # 仿真1秒
    print("运行仿真（1秒）...")
    t, states = mav.simulate((0, 1), forces, moments, dt=0.01)

    print(f"仿真完成，共 {len(t)} 个时间步")
    print(f"初始位置: pn={states[0, 0]:.2f}, pe={states[0, 1]:.2f}, pd={states[0, 2]:.2f}")
    print(f"最终位置: pn={states[-1, 0]:.2f}, pe={states[-1, 1]:.2f}, pd={states[-1, 2]:.2f}")
    print(f"初始速度: u={states[0, 3]:.2f}, v={states[0, 4]:.2f}, w={states[0, 5]:.2f}")
    print(f"最终速度: u={states[-1, 3]:.2f}, v={states[-1, 4]:.2f}, w={states[-1, 5]:.2f}")
