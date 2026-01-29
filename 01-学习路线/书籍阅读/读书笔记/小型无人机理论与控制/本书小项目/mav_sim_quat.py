"""
基于四元数的MAV仿真主程序
集成四元数动力学、力与力矩模型、风速模型

特性:
- 内部使用四元数，无万向锁
- 兼容欧拉角输入/输出
- 自动四元数归一化
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from mav_params import ZagiParams, UAVParams
from mav_dynamics_quat import MAVDynamicsQuat, StateIndexQuat, state_euler_to_quat, state_quat_to_euler
from forces_moments_quat import ForcesAndMomentsQuat, compute_trim_throttle
from wind_model import WindModel
from quaternion import quaternion_to_euler, euler_to_quaternion, quaternion_norm_error


class MAVSimulationQuat:
    """基于四元数的MAV仿真类"""

    def __init__(self, params=ZagiParams, dt=0.01, lambda_quat=1000.0):
        """
        初始化仿真

        参数:
            params: 飞机参数类
            dt: 仿真时间步长 (s)
            lambda_quat: 四元数正交控制增益
        """
        self.params = params
        self.dt = dt

        # 创建动力学模型（四元数）
        self.dynamics = MAVDynamicsQuat(
            mass=params.m,
            Jx=params.Jx,
            Jy=params.Jy,
            Jz=params.Jz,
            Jxz=params.Jxz,
            lambda_quat=lambda_quat
        )

        # 创建力与力矩计算器（四元数版本）
        self.forces_moments = ForcesAndMomentsQuat(params)

        # 风速模型
        self.wind_model = None

        # 控制输入 [delta_e, delta_a, delta_r, delta_t]
        self.controls = np.array([0.0, 0.0, 0.0, 0.5])

        # 仿真历史
        self._clear_history()

    def _clear_history(self):
        """清空历史记录"""
        self.time_history = []
        self.state_history = []         # 四元数状态 (13维)
        self.state_euler_history = []   # 欧拉角状态 (12维)
        self.forces_history = []
        self.moments_history = []
        self.Va_history = []
        self.alpha_history = []
        self.beta_history = []
        self.wind_history = []
        self.quat_norm_error_history = []

    def set_initial_state(self, state):
        """
        设置初始状态

        参数:
            state: 状态向量
                - 12维: 欧拉角形式 [pn,pe,pd,u,v,w,φ,θ,ψ,p,q,r]
                - 13维: 四元数形式 [pn,pe,pd,u,v,w,e0,e1,e2,e3,p,q,r]
        """
        self.dynamics.set_state(state)

    def set_controls(self, delta_e=0, delta_a=0, delta_r=0, delta_t=0.5):
        """设置控制输入"""
        self.controls = np.array([delta_e, delta_a, delta_r, delta_t])

    def set_wind(self, steady_wind_ned=None, turbulence_level=None, Va=20):
        """
        设置风速模型

        参数:
            steady_wind_ned: 稳定风 [wn, we, wd] (m/s)
            turbulence_level: 湍流等级
            Va: 名义空速
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

        # 从四元数状态获取欧拉角
        phi, theta, psi = quaternion_to_euler(state[6:10])
        return self.wind_model.update(phi, theta, psi)

    def simulate(self, t_span, controls_func=None):
        """
        运行仿真

        参数:
            t_span: 时间范围 (t_start, t_end)
            controls_func: 控制函数 f(t) -> [delta_e, delta_a, delta_r, delta_t]

        返回:
            t: 时间数组
            states: 状态历史（四元数形式）
        """
        t_eval = np.arange(t_span[0], t_span[1], self.dt)

        # 重置风速模型
        if self.wind_model is not None:
            self.wind_model.reset()

        # 清空历史
        self._clear_history()

        # 获取初始状态
        state = self.dynamics.get_state()

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
                state, wind_body, controls, use_quaternion=True
            )

            # 记录历史
            self.time_history.append(t)
            self.state_history.append(state.copy())
            self.state_euler_history.append(state_quat_to_euler(state))
            self.forces_history.append(forces.copy())
            self.moments_history.append(moments.copy())
            self.Va_history.append(Va)
            self.alpha_history.append(alpha)
            self.beta_history.append(beta)
            self.wind_history.append(wind_ned.copy())
            self.quat_norm_error_history.append(quaternion_norm_error(state[6:10]))

            # RK4积分
            state = self.dynamics.rk4_step(t, forces, moments, self.dt)

        # 转换为数组
        self.time_history = np.array(self.time_history)
        self.state_history = np.array(self.state_history)
        self.state_euler_history = np.array(self.state_euler_history)
        self.forces_history = np.array(self.forces_history)
        self.moments_history = np.array(self.moments_history)
        self.Va_history = np.array(self.Va_history)
        self.alpha_history = np.array(self.alpha_history)
        self.beta_history = np.array(self.beta_history)
        self.wind_history = np.array(self.wind_history)
        self.quat_norm_error_history = np.array(self.quat_norm_error_history)

        return self.time_history, self.state_history

    def get_euler_angles(self):
        """获取欧拉角历史 (phi, theta, psi)"""
        return self.state_euler_history[:, 6:9]

    def get_quaternion_history(self):
        """获取四元数历史"""
        return self.state_history[:, 6:10]

    def plot_results(self, figsize=(16, 12)):
        """绘制仿真结果"""
        t = self.time_history
        s = self.state_euler_history  # 使用欧拉角形式便于显示

        fig, axes = plt.subplots(4, 3, figsize=figsize)
        fig.suptitle('MAV Simulation Results (Quaternion-based)', fontsize=14)

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

        # 四元数
        ax = axes[1, 2]
        q = self.state_history[:, 6:10]
        ax.plot(t, q[:, 0], label='e0')
        ax.plot(t, q[:, 1], label='e1')
        ax.plot(t, q[:, 2], label='e2')
        ax.plot(t, q[:, 3], label='e3')
        ax.set_ylabel('Quaternion')
        ax.set_title('Quaternion Components')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 力
        ax = axes[2, 0]
        ax.plot(t, self.forces_history[:, 0], label='fx')
        ax.plot(t, self.forces_history[:, 1], label='fy')
        ax.plot(t, self.forces_history[:, 2], label='fz')
        ax.set_ylabel('Force (N)')
        ax.set_title('Total Forces')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 力矩
        ax = axes[2, 1]
        ax.plot(t, self.moments_history[:, 0], label='l')
        ax.plot(t, self.moments_history[:, 1], label='m')
        ax.plot(t, self.moments_history[:, 2], label='n')
        ax.set_ylabel('Moment (N·m)')
        ax.set_title('Total Moments')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # 四元数范数误差
        ax = axes[2, 2]
        ax.semilogy(t, self.quat_norm_error_history + 1e-16)
        ax.set_ylabel('Norm Error')
        ax.set_title('Quaternion Normalization Error')
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

        # 地面轨迹
        ax = axes[3, 2]
        ax.plot(s[:, 1], s[:, 0])
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_title('Ground Track')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()


def demo_quaternion_simulation():
    """演示四元数仿真"""
    print("=" * 50)
    print("四元数MAV仿真演示")
    print("=" * 50)

    sim = MAVSimulationQuat(ZagiParams, dt=0.01)

    # 初始状态（使用欧拉角输入）
    Va = 25.0
    initial_state = np.zeros(12)
    initial_state[3] = Va  # u = Va
    initial_state[2] = -100  # pd = -100 (高度100m)
    initial_state[7] = np.deg2rad(2)  # θ = 2°
    sim.set_initial_state(initial_state)

    # 控制
    delta_t = compute_trim_throttle(ZagiParams, Va)
    print(f"估算配平油门: {delta_t:.3f}")
    sim.set_controls(delta_e=np.deg2rad(-2), delta_t=delta_t)

    # 无风
    sim.set_wind(steady_wind_ned=None, turbulence_level=None)

    # 仿真
    print("运行仿真...")
    sim.simulate((0, 10))

    # 检查四元数归一化
    max_norm_error = np.max(sim.quat_norm_error_history)
    print(f"四元数最大范数误差: {max_norm_error:.2e}")

    # 绘图
    sim.plot_results()


if __name__ == "__main__":
    demo_quaternion_simulation()
