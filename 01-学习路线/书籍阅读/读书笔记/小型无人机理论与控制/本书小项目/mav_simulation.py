"""
MAV六自由度仿真主程序
连接动力学模型和3D可视化动画
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

from mav_params import ZagiParams, UAVParams, g
from mav_dynamics import MAVDynamics, create_mav_from_params, StateIndex


class MAVViewer:
    """MAV 3D可视化器"""

    def __init__(self, scale=1.0):
        """
        初始化可视化器

        参数:
            scale: 飞行器模型缩放比例
        """
        self.scale = scale
        self.fig = None
        self.ax = None

        # 飞行器几何参数（缩放后）
        # 机体坐标系：x指向机头，y指向右翼，z指向机腹（向下）
        self.fuse_l1 = 1.5 * scale   # 机头到重心距离
        self.fuse_l2 = 1.0 * scale   # 重心到机身后端距离
        self.fuse_l3 = 4.0 * scale   # 机身后端到尾锥尖距离
        self.fuse_w = 1.0 * scale    # 机身宽度
        self.fuse_h = 1.0 * scale    # 机身高度
        self.wing_w = 6.0 * scale    # 翼展
        self.wing_l = 1.5 * scale    # 翼弦
        self.tail_w = 2.5 * scale    # 尾翼展
        self.tail_l = 1.0 * scale    # 尾翼弦
        self.tail_h = 1.5 * scale    # 垂尾高

    def _build_vertices(self):
        """
        构建飞行器顶点（机体坐标系，原点在重心）

        机体坐标系定义：
        - x轴：指向机头（正前方）
        - y轴：指向右翼
        - z轴：指向机腹（向下）
        """
        # 机头锥尖（x正方向）
        nose = np.array([self.fuse_l1, 0, 0])

        # 机尾锥尖（x负方向）
        tail = np.array([-(self.fuse_l2 + self.fuse_l3), 0, 0])

        # 机身基准矩形（在重心后方一点）
        w, h = self.fuse_w / 2, self.fuse_h / 2
        base_x = -self.fuse_l2
        base = np.array([
            [base_x,  w, -h],  # 右上
            [base_x, -w, -h],  # 左上
            [base_x, -w,  h],  # 左下
            [base_x,  w,  h],  # 右下
        ])

        # 机翼顶点（在重心附近，x=0左右）
        wing_x_front = 0.3 * self.fuse_l1
        wing_x_back = wing_x_front - self.wing_l
        wing = np.array([
            [wing_x_front,  self.wing_w / 2, 0],  # 右翼前缘
            [wing_x_front, -self.wing_w / 2, 0],  # 左翼前缘
            [wing_x_back,  -self.wing_w / 2, 0],  # 左翼后缘
            [wing_x_back,   self.wing_w / 2, 0],  # 右翼后缘
        ])

        # 水平尾翼顶点
        htail_x_front = -(self.fuse_l2 + self.fuse_l3 * 0.6)
        htail_x_back = htail_x_front - self.tail_l
        htail = np.array([
            [htail_x_front,  self.tail_w / 2, 0],
            [htail_x_front, -self.tail_w / 2, 0],
            [htail_x_back,  -self.tail_w / 2, 0],
            [htail_x_back,   self.tail_w / 2, 0],
        ])

        # 垂直尾翼顶点（向上，z负方向）
        vtail = np.array([
            [htail_x_front, 0, 0],
            [htail_x_back,  0, 0],
            [htail_x_back,  0, -self.tail_h],  # 向上（z负）
        ])

        return {
            "nose": nose,
            "tail": tail,
            "base": base,
            "wing": wing,
            "htail": htail,
            "vtail": vtail
        }

    def _rotation_matrix(self, phi, theta, psi):
        """旋转矩阵（从机体系到NED惯性系）"""
        c_phi, s_phi = np.cos(phi), np.sin(phi)
        c_theta, s_theta = np.cos(theta), np.sin(theta)
        c_psi, s_psi = np.cos(psi), np.sin(psi)

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

    def _transform_points(self, points, phi, theta, psi, position):
        """将点从机体系变换到惯性系"""
        R = self._rotation_matrix(phi, theta, psi)
        if points.ndim == 1:
            return R @ points + position
        else:
            return (R @ points.T).T + position

    def _draw_polyline(self, ax, pts, close=True, **kwargs):
        """绘制多边形线框"""
        xs = list(pts[:, 0])
        ys = list(pts[:, 1])
        zs = list(pts[:, 2])
        if close:
            xs.append(pts[0, 0])
            ys.append(pts[0, 1])
            zs.append(pts[0, 2])
        ax.plot(xs, ys, zs, **kwargs)

    def _draw_pyramid(self, ax, apex, base_pts, **kwargs):
        """绘制四棱锥"""
        self._draw_polyline(ax, base_pts, close=True, **kwargs)
        for bp in base_pts:
            ax.plot([apex[0], bp[0]], [apex[1], bp[1]], [apex[2], bp[2]], **kwargs)

    def draw_mav(self, ax, state, color="#1f77b4"):
        """
        绘制MAV

        参数:
            ax: matplotlib 3D轴
            state: 状态向量 [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
            color: 线框颜色
        """
        pn, pe, pd = state[0:3]
        phi, theta, psi = state[6:9]
        position = np.array([pn, pe, pd])

        v = self._build_vertices()

        # 变换所有顶点
        nose_t = self._transform_points(v["nose"], phi, theta, psi, position)
        tail_t = self._transform_points(v["tail"], phi, theta, psi, position)
        base_t = self._transform_points(v["base"], phi, theta, psi, position)
        wing_t = self._transform_points(v["wing"], phi, theta, psi, position)
        htail_t = self._transform_points(v["htail"], phi, theta, psi, position)
        vtail_t = self._transform_points(v["vtail"], phi, theta, psi, position)

        # 绘制机身
        self._draw_pyramid(ax, nose_t, base_t, color=color, linewidth=1.2)
        self._draw_pyramid(ax, tail_t, base_t, color=color, linewidth=1.2)

        # 绘制机翼和尾翼
        self._draw_polyline(ax, wing_t, close=True, color=color, linewidth=1.2)
        self._draw_polyline(ax, htail_t, close=True, color=color, linewidth=1.2)
        self._draw_polyline(ax, vtail_t, close=True, color=color, linewidth=1.2)


class MAVSimulation:
    """MAV仿真与可视化"""

    def __init__(self, mav_dynamics, viewer_scale=1.0):
        """
        初始化仿真

        参数:
            mav_dynamics: MAVDynamics实例
            viewer_scale: 可视化缩放比例
        """
        self.mav = mav_dynamics
        self.viewer = MAVViewer(scale=viewer_scale)

        self.time_history = None
        self.state_history = None

    def run(self, t_span, forces_func, moments_func, dt=0.01):
        """
        运行仿真

        参数:
            t_span: 时间范围 (t_start, t_end)
            forces_func: 力函数 f(t, state) -> [fx, fy, fz]
            moments_func: 力矩函数 f(t, state) -> [l, m, n]
            dt: 时间步长
        """
        self.time_history, self.state_history = self.mav.simulate(
            t_span, forces_func, moments_func, dt
        )
        return self.time_history, self.state_history

    def animate(self, speed=1.0, trail=True, figsize=(14, 6)):
        """
        播放动画

        参数:
            speed: 播放速度倍率
            trail: 是否显示轨迹
            figsize: 图形大小
        """
        if self.state_history is None:
            raise ValueError("请先运行仿真 (run 方法)")

        fig = plt.figure(figsize=figsize)

        # 3D视图
        ax3d = fig.add_subplot(121, projection="3d")

        # 状态曲线图
        ax2d = fig.add_subplot(122)

        states = self.state_history
        times = self.time_history

        # 计算坐标范围，确保飞机模型可见
        model_size = self.viewer.scale * 8  # 模型大致尺寸

        pn_range = [states[:, 0].min(), states[:, 0].max()]
        pe_range = [states[:, 1].min(), states[:, 1].max()]
        pd_range = [states[:, 2].min(), states[:, 2].max()]

        # 添加边距，确保模型不会超出边界
        margin = max(model_size, 10)
        pn_range = [pn_range[0] - margin, pn_range[1] + margin]
        pe_range = [pe_range[0] - margin, pe_range[1] + margin]
        pd_range = [pd_range[0] - margin, pd_range[1] + margin]

        # 确保各轴范围相等（保持比例）
        max_range = max(
            pn_range[1] - pn_range[0],
            pe_range[1] - pe_range[0],
            pd_range[1] - pd_range[0]
        )

        pn_mid = (pn_range[0] + pn_range[1]) / 2
        pe_mid = (pe_range[0] + pe_range[1]) / 2
        pd_mid = (pd_range[0] + pd_range[1]) / 2

        pn_range = [pn_mid - max_range/2, pn_mid + max_range/2]
        pe_range = [pe_mid - max_range/2, pe_mid + max_range/2]
        pd_range = [pd_mid - max_range/2, pd_mid + max_range/2]

        def update(frame):
            ax3d.cla()

            # 设置3D视图
            ax3d.set_xlabel("North (m)")
            ax3d.set_ylabel("East (m)")
            ax3d.set_zlabel("Down (m)")
            ax3d.set_xlim(pn_range)
            ax3d.set_ylim(pe_range)
            ax3d.set_zlim(pd_range)
            ax3d.invert_zaxis()  # NED坐标系，向下为正
            ax3d.set_title(f"MAV 3D View (t = {times[frame]:.2f}s)")
            ax3d.set_box_aspect([1, 1, 1])  # 保持坐标轴比例相等

            # 绘制轨迹
            if trail and frame > 0:
                ax3d.plot(
                    states[:frame, 0],
                    states[:frame, 1],
                    states[:frame, 2],
                    "b-", alpha=0.3, linewidth=0.5
                )

            # 绘制MAV
            self.viewer.draw_mav(ax3d, states[frame])

            # 更新状态曲线图
            ax2d.cla()
            ax2d.set_xlabel("Time (s)")
            ax2d.set_ylabel("Value")
            ax2d.set_title("State History")

            # 绘制欧拉角（转换为度）
            ax2d.plot(times[:frame+1], np.rad2deg(states[:frame+1, 6]), "r-", label="φ (roll)")
            ax2d.plot(times[:frame+1], np.rad2deg(states[:frame+1, 7]), "g-", label="θ (pitch)")
            ax2d.plot(times[:frame+1], np.rad2deg(states[:frame+1, 8]), "b-", label="ψ (yaw)")
            ax2d.legend(loc="upper right")
            ax2d.grid(True, alpha=0.3)
            ax2d.set_xlim(times[0], times[-1])
            ax2d.set_ylim(-90, 90)

        # 计算帧间隔（考虑播放速度）
        dt = times[1] - times[0] if len(times) > 1 else 0.01
        interval = int(dt * 1000 / speed)

        ani = FuncAnimation(fig, update, frames=len(times), interval=interval, repeat=True)

        plt.tight_layout()
        plt.show()

        return ani

    def plot_results(self, figsize=(14, 10)):
        """绘制仿真结果"""
        if self.state_history is None:
            raise ValueError("请先运行仿真")

        t = self.time_history
        s = self.state_history

        fig, axes = plt.subplots(3, 2, figsize=figsize)

        # 位置
        ax = axes[0, 0]
        ax.plot(t, s[:, 0], label="pn (North)")
        ax.plot(t, s[:, 1], label="pe (East)")
        ax.plot(t, -s[:, 2], label="h (Altitude)")  # 高度 = -pd
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (m)")
        ax.set_title("Position")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 速度
        ax = axes[0, 1]
        ax.plot(t, s[:, 3], label="u")
        ax.plot(t, s[:, 4], label="v")
        ax.plot(t, s[:, 5], label="w")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.set_title("Body Velocities")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 欧拉角
        ax = axes[1, 0]
        ax.plot(t, np.rad2deg(s[:, 6]), label="φ (roll)")
        ax.plot(t, np.rad2deg(s[:, 7]), label="θ (pitch)")
        ax.plot(t, np.rad2deg(s[:, 8]), label="ψ (yaw)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (deg)")
        ax.set_title("Euler Angles")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 角速度
        ax = axes[1, 1]
        ax.plot(t, np.rad2deg(s[:, 9]), label="p (roll rate)")
        ax.plot(t, np.rad2deg(s[:, 10]), label="q (pitch rate)")
        ax.plot(t, np.rad2deg(s[:, 11]), label="r (yaw rate)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angular Rate (deg/s)")
        ax.set_title("Angular Velocities")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 3D轨迹
        ax = axes[2, 0]
        ax.plot(s[:, 0], s[:, 1])
        ax.set_xlabel("North (m)")
        ax.set_ylabel("East (m)")
        ax.set_title("Ground Track")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)

        # 高度剖面
        ax = axes[2, 1]
        ax.plot(t, -s[:, 2])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Altitude (m)")
        ax.set_title("Altitude Profile")
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()


def demo_gravity_only():
    """演示：仅重力作用下的自由落体"""
    print("=== 演示：重力作用下的运动 ===")

    # 创建MAV
    mav = create_mav_from_params(ZagiParams)

    # 初始状态：向前飞行，有一定俯仰角
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 20.0      # 前向速度 20 m/s
    initial_state[StateIndex.PD] = -100.0   # 高度 100m
    initial_state[StateIndex.THETA] = np.deg2rad(5)  # 俯仰角 5度
    mav.set_state(initial_state)

    # 力函数：仅重力
    def forces(t, state):
        phi = state[StateIndex.PHI]
        theta = state[StateIndex.THETA]
        return mav.get_gravity_force_body(phi, theta)

    # 力矩函数：无力矩
    def moments(t, state):
        return np.array([0.0, 0.0, 0.0])

    # 创建仿真
    sim = MAVSimulation(mav, viewer_scale=2.0)

    # 运行仿真
    sim.run((0, 5), forces, moments, dt=0.02)

    # 显示结果
    sim.plot_results()

    # 播放动画
    sim.animate(speed=1.0)


if __name__ == "__main__":
    demo_gravity_only()
