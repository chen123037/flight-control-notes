"""
第四章 MAV仿真主程序
集成力与力矩模型、风速模型和动力学

基于第三章的动力学模型，添加：
- 完整的气动力计算
- 风速模型（稳定风+德莱登干扰风）
- 控制面输入
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

from mav_params import ZagiParams, UAVParams, compute_gamma_coefficients, g
from mav_dynamics import MAVDynamics, StateIndex
from forces_moments import ForcesAndMoments, compute_trim_throttle
from wind_model import WindModel


class MAVSimulationCh4:
    """第四章MAV仿真类"""

    def __init__(self, params=ZagiParams, dt=0.01):
        """
        初始化仿真

        参数:
            params: 飞机参数类
            dt: 仿真时间步长 (s)
        """
        self.params = params
        self.dt = dt

        # 创建动力学模型
        self.dynamics = MAVDynamics(
            mass=params.m,
            Jx=params.Jx,
            Jy=params.Jy,
            Jz=params.Jz,
            Jxz=params.Jxz
        )

        # 创建力与力矩计算器
        self.forces_moments = ForcesAndMoments(params)

        # 风速模型（默认无风）
        self.wind_model = None

        # 控制输入 [delta_e, delta_a, delta_r, delta_t]
        self.controls = np.array([0.0, 0.0, 0.0, 0.5])

        # 仿真历史
        self.time_history = []
        self.state_history = []
        self.forces_history = []
        self.moments_history = []
        self.Va_history = []
        self.alpha_history = []
        self.beta_history = []
        self.wind_history = []

    def set_initial_state(self, state):
        """设置初始状态"""
        self.dynamics.set_state(state)

    def set_controls(self, delta_e=0, delta_a=0, delta_r=0, delta_t=0.5):
        """设置控制输入"""
        self.controls = np.array([delta_e, delta_a, delta_r, delta_t])

    def set_wind(self, steady_wind_ned=None, turbulence_level=None, Va=20):
        """
        设置风速模型

        参数:
            steady_wind_ned: 稳定风 [wn, we, wd] (m/s)
            turbulence_level: 湍流等级 ('light_low', 'moderate_low', etc.)
            Va: 名义空速，用于德莱登模型
        """
        self.wind_model = WindModel(
            Va=Va,
            steady_wind_ned=steady_wind_ned,
            turbulence_level=turbulence_level,
            dt=self.dt
        )

    def _get_wind(self, state):
        """获取当前风速"""
        if self.wind_model is None:
            return np.zeros(3), np.zeros(3)

        phi, theta, psi = state[6:9]
        return self.wind_model.update(phi, theta, psi)

    def _equations_of_motion(self, t, state, controls_func=None):
        """
        运动方程（ODE右端函数）

        参数:
            t: 时间
            state: 状态向量
            controls_func: 控制函数 f(t) -> [delta_e, delta_a, delta_r, delta_t]
        """
        # 获取控制输入
        if controls_func is not None:
            controls = controls_func(t)
        else:
            controls = self.controls

        # 获取风速
        wind_body, _ = self._get_wind(state)

        # 计算力与力矩
        forces, moments, Va, alpha, beta = self.forces_moments.compute(
            state, wind_body, controls
        )

        # 调用动力学方程
        return self.dynamics.equations_of_motion(t, state, forces, moments)

    def simulate(self, t_span, controls_func=None):
        """
        运行仿真

        参数:
            t_span: 时间范围 (t_start, t_end)
            controls_func: 控制函数 f(t) -> [delta_e, delta_a, delta_r, delta_t]
                          如果为None，使用固定控制输入

        返回:
            t: 时间数组
            states: 状态历史
        """
        t_eval = np.arange(t_span[0], t_span[1], self.dt)

        # 重置风速模型
        if self.wind_model is not None:
            self.wind_model.reset()

        # 清空历史
        self.time_history = []
        self.state_history = []
        self.forces_history = []
        self.moments_history = []
        self.Va_history = []
        self.alpha_history = []
        self.beta_history = []
        self.wind_history = []

        # 使用手动时间步进（为了记录中间变量）
        state = self.dynamics.get_state()
        t_current = t_span[0]

        for t in t_eval:
            # 获取控制
            if controls_func is not None:
                controls = controls_func(t)
            else:
                controls = self.controls

            # 获取风速
            wind_body, wind_ned = self._get_wind(state)

            # 计算力与力矩
            forces, moments, Va, alpha, beta = self.forces_moments.compute(
                state, wind_body, controls
            )

            # 记录历史
            self.time_history.append(t)
            self.state_history.append(state.copy())
            self.forces_history.append(forces.copy())
            self.moments_history.append(moments.copy())
            self.Va_history.append(Va)
            self.alpha_history.append(alpha)
            self.beta_history.append(beta)
            self.wind_history.append(wind_ned.copy())

            # RK4积分
            k1 = self.dynamics.equations_of_motion(t, state, forces, moments)
            k2 = self.dynamics.equations_of_motion(t + self.dt/2, state + self.dt/2 * k1, forces, moments)
            k3 = self.dynamics.equations_of_motion(t + self.dt/2, state + self.dt/2 * k2, forces, moments)
            k4 = self.dynamics.equations_of_motion(t + self.dt, state + self.dt * k3, forces, moments)
            state = state + self.dt / 6 * (k1 + 2*k2 + 2*k3 + k4)

        # 转换为数组
        self.time_history = np.array(self.time_history)
        self.state_history = np.array(self.state_history)
        self.forces_history = np.array(self.forces_history)
        self.moments_history = np.array(self.moments_history)
        self.Va_history = np.array(self.Va_history)
        self.alpha_history = np.array(self.alpha_history)
        self.beta_history = np.array(self.beta_history)
        self.wind_history = np.array(self.wind_history)

        return self.time_history, self.state_history

    def plot_results(self, figsize=(16, 12)):
        """绘制仿真结果"""
        t = self.time_history
        s = self.state_history

        fig, axes = plt.subplots(4, 3, figsize=figsize)
        fig.suptitle('MAV Simulation Results (Chapter 4)', fontsize=14)

        # 位置
        ax = axes[0, 0]
        ax.plot(t, s[:, 0], label='pn (North)')
        ax.plot(t, s[:, 1], label='pe (East)')
        ax.plot(t, -s[:, 2], label='h (Altitude)')
        ax.set_ylabel('Position (m)')
        ax.set_title('Position')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 速度
        ax = axes[0, 1]
        ax.plot(t, s[:, 3], label='u')
        ax.plot(t, s[:, 4], label='v')
        ax.plot(t, s[:, 5], label='w')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Body Velocities')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 空速、攻角、侧滑角
        ax = axes[0, 2]
        ax.plot(t, self.Va_history, label='Va')
        ax.set_ylabel('Airspeed (m/s)')
        ax.set_title('Airspeed')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        ax2 = ax.twinx()
        ax2.plot(t, np.rad2deg(self.alpha_history), 'r-', label='α', alpha=0.7)
        ax2.plot(t, np.rad2deg(self.beta_history), 'g-', label='β', alpha=0.7)
        ax2.set_ylabel('Angle (deg)')
        ax2.legend(fontsize=8, loc='upper right')

        # 欧拉角
        ax = axes[1, 0]
        ax.plot(t, np.rad2deg(s[:, 6]), label='φ (roll)')
        ax.plot(t, np.rad2deg(s[:, 7]), label='θ (pitch)')
        ax.plot(t, np.rad2deg(s[:, 8]), label='ψ (yaw)')
        ax.set_ylabel('Angle (deg)')
        ax.set_title('Euler Angles')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 角速度
        ax = axes[1, 1]
        ax.plot(t, np.rad2deg(s[:, 9]), label='p')
        ax.plot(t, np.rad2deg(s[:, 10]), label='q')
        ax.plot(t, np.rad2deg(s[:, 11]), label='r')
        ax.set_ylabel('Angular Rate (deg/s)')
        ax.set_title('Angular Velocities')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 力
        ax = axes[1, 2]
        ax.plot(t, self.forces_history[:, 0], label='fx')
        ax.plot(t, self.forces_history[:, 1], label='fy')
        ax.plot(t, self.forces_history[:, 2], label='fz')
        ax.set_ylabel('Force (N)')
        ax.set_title('Total Forces')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 力矩
        ax = axes[2, 0]
        ax.plot(t, self.moments_history[:, 0], label='l')
        ax.plot(t, self.moments_history[:, 1], label='m')
        ax.plot(t, self.moments_history[:, 2], label='n')
        ax.set_ylabel('Moment (N·m)')
        ax.set_title('Total Moments')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 风速
        ax = axes[2, 1]
        ax.plot(t, self.wind_history[:, 0], label='wn')
        ax.plot(t, self.wind_history[:, 1], label='we')
        ax.plot(t, self.wind_history[:, 2], label='wd')
        ax.set_ylabel('Wind (m/s)')
        ax.set_title('Wind (NED)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 3D轨迹
        ax = axes[2, 2]
        ax.plot(s[:, 1], s[:, 0])
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_title('Ground Track')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)

        # 高度剖面
        ax = axes[3, 0]
        ax.plot(t, -s[:, 2])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Altitude (m)')
        ax.set_title('Altitude Profile')
        ax.grid(True, alpha=0.3)

        # 3D轨迹
        ax = axes[3, 1]
        ax.remove()
        ax = fig.add_subplot(4, 3, 11, projection='3d')
        ax.plot(s[:, 0], s[:, 1], -s[:, 2])
        ax.set_xlabel('North')
        ax.set_ylabel('East')
        ax.set_zlabel('Altitude')
        ax.set_title('3D Trajectory')

        # 空白
        axes[3, 2].axis('off')

        plt.tight_layout()
        plt.show()


def demo_level_flight():
    """演示：平飞"""
    print("=== 演示：平飞 ===")

    sim = MAVSimulationCh4(ZagiParams, dt=0.01)

    # 初始状态
    Va = 25.0
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = Va
    initial_state[StateIndex.PD] = -100  # 高度100m
    initial_state[StateIndex.THETA] = np.deg2rad(2)  # 小俯仰角
    sim.set_initial_state(initial_state)

    # 估算配平油门
    delta_t = compute_trim_throttle(ZagiParams, Va)
    print(f"估算配平油门: {delta_t:.3f}")

    # 设置控制（需要一些升降舵配平）
    sim.set_controls(delta_e=np.deg2rad(-2), delta_t=delta_t)

    # 无风
    sim.set_wind(steady_wind_ned=None, turbulence_level=None)

    # 仿真10秒
    sim.simulate((0, 10))

    # 绘图
    sim.plot_results()


def demo_with_wind():
    """演示：有风飞行"""
    print("=== 演示：有风飞行 ===")

    sim = MAVSimulationCh4(ZagiParams, dt=0.01)

    # 初始状态
    Va = 25.0
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = Va
    initial_state[StateIndex.PD] = -100
    initial_state[StateIndex.THETA] = np.deg2rad(2)
    sim.set_initial_state(initial_state)

    # 控制
    delta_t = compute_trim_throttle(ZagiParams, Va)
    sim.set_controls(delta_e=np.deg2rad(-2), delta_t=delta_t)

    # 设置风：稳定风 + 轻度干扰
    sim.set_wind(
        steady_wind_ned=[5, 2, 0],  # 北风5m/s，东风2m/s
        turbulence_level='light_low',
        Va=Va
    )

    # 仿真15秒
    sim.simulate((0, 15))

    # 绘图
    sim.plot_results()


def demo_control_response():
    """演示：控制面响应"""
    print("=== 演示：控制面响应 ===")

    sim = MAVSimulationCh4(ZagiParams, dt=0.01)

    # 初始状态
    Va = 25.0
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = Va
    initial_state[StateIndex.PD] = -100
    sim.set_initial_state(initial_state)

    # 控制函数：在不同时间施加不同控制
    delta_t = compute_trim_throttle(ZagiParams, Va)

    def controls_func(t):
        delta_e = 0
        delta_a = 0
        delta_r = 0

        if 2 < t < 4:
            # 升降舵：抬头
            delta_e = np.deg2rad(-5)
        elif 6 < t < 8:
            # 副翼：右滚
            delta_a = np.deg2rad(10)
        elif 10 < t < 12:
            # 方向舵（Zagi没有，但测试）
            delta_r = np.deg2rad(5)

        return [delta_e, delta_a, delta_r, delta_t]

    # 无风
    sim.set_wind(steady_wind_ned=None, turbulence_level=None)

    # 仿真15秒
    sim.simulate((0, 15), controls_func)

    # 绘图
    sim.plot_results()


if __name__ == "__main__":
    print("=" * 50)
    print("第四章 MAV仿真")
    print("=" * 50)
    print("\n选择演示：")
    print("1. 平飞（无风）")
    print("2. 有风飞行（稳定风+干扰风）")
    print("3. 控制面响应测试")
    print("0. 退出")

    while True:
        try:
            choice = input("\n请选择 (0-3): ").strip()
            if choice == "0":
                break
            elif choice == "1":
                demo_level_flight()
            elif choice == "2":
                demo_with_wind()
            elif choice == "3":
                demo_control_response()
            else:
                print("无效选择")
        except KeyboardInterrupt:
            break
