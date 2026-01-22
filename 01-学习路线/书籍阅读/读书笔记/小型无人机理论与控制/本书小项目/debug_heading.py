"""
调试机头方向问题
从俯视图清楚地显示：
1. 飞机位置和轨迹
2. 机头方向（红色箭头）
3. 速度方向（绿色箭头）
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

from mav_params import ZagiParams
from mav_dynamics import create_mav_from_params, StateIndex


def debug_straight_flight():
    """调试：直线飞行，机头应该指向北"""
    print("=== 调试：直线飞行 ===")
    print("预期：机头（红箭头）和速度（绿箭头）都指向北")

    mav = create_mav_from_params(ZagiParams)

    # 直线向北飞行，无力矩
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 20.0
    initial_state[StateIndex.PD] = -50.0
    mav.set_state(initial_state)

    def forces(t, state):
        return mav.get_gravity_force_body(state[6], state[7])

    def moments(t, state):
        return np.zeros(3)

    t, states = mav.simulate((0, 3), forces, moments, dt=0.1)

    # 绘制俯视图
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Top View - Straight Flight\nRed=Nose direction, Green=Velocity direction')
    ax.grid(True, alpha=0.3)

    # 绘制轨迹
    ax.plot(states[:, 1], states[:, 0], 'b-', linewidth=2, label='Trajectory')

    # 每隔几帧绘制方向箭头
    arrow_length = 5
    for i in range(0, len(states), 5):
        pn, pe = states[i, 0], states[i, 1]
        psi = states[i, 8]
        u, v = states[i, 3], states[i, 4]

        # 机头方向（由psi决定）
        # psi=0 时机头指向北，psi>0 时机头指向东
        nose_north = np.cos(psi) * arrow_length
        nose_east = np.sin(psi) * arrow_length
        ax.arrow(pe, pn, nose_east, nose_north,
                head_width=1, head_length=0.5, fc='red', ec='red', alpha=0.7)

        # 速度方向（惯性系）
        # 需要将机体系速度转换到惯性系
        c_psi, s_psi = np.cos(psi), np.sin(psi)
        vel_north = c_psi * u - s_psi * v  # 简化（假设phi=theta=0）
        vel_east = s_psi * u + c_psi * v
        vel_mag = np.sqrt(vel_north**2 + vel_east**2)
        if vel_mag > 0.1:
            vel_north_norm = vel_north / vel_mag * arrow_length
            vel_east_norm = vel_east / vel_mag * arrow_length
            ax.arrow(pe, pn, vel_east_norm, vel_north_norm,
                    head_width=1, head_length=0.5, fc='green', ec='green', alpha=0.7)

    ax.legend()
    plt.show()

    print(f"\n数据检查：")
    print(f"初始 psi = {np.rad2deg(states[0, 8]):.1f}°")
    print(f"最终 psi = {np.rad2deg(states[-1, 8]):.1f}°")
    print(f"初始位置: North={states[0, 0]:.1f}, East={states[0, 1]:.1f}")
    print(f"最终位置: North={states[-1, 0]:.1f}, East={states[-1, 1]:.1f}")


def debug_yaw_rotation():
    """调试：施加偏航力矩"""
    print("\n=== 调试：施加偏航力矩 ===")
    print("预期：飞机向右转弯，机头和速度方向应该一致")

    mav = create_mav_from_params(ZagiParams)

    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 15.0
    initial_state[StateIndex.PD] = -50.0
    mav.set_state(initial_state)

    def forces(t, state):
        fg = mav.get_gravity_force_body(state[6], state[7])
        return fg + np.array([2.0, 0, 0])  # 加点推力

    def moments(t, state):
        return np.array([0.0, 0.0, 0.3])  # 偏航力矩

    t, states = mav.simulate((0, 6), forces, moments, dt=0.1)

    # 绘制俯视图
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    # 左图：轨迹和方向
    ax = axes[0]
    ax.set_aspect('equal')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Top View - Yaw Test\nRed=Nose, Green=Velocity')
    ax.grid(True, alpha=0.3)

    ax.plot(states[:, 1], states[:, 0], 'b-', linewidth=2, label='Trajectory')

    arrow_length = 3
    for i in range(0, len(states), 8):
        pn, pe = states[i, 0], states[i, 1]
        psi = states[i, 8]
        u, v = states[i, 3], states[i, 4]

        # 机头方向
        nose_north = np.cos(psi) * arrow_length
        nose_east = np.sin(psi) * arrow_length
        ax.arrow(pe, pn, nose_east, nose_north,
                head_width=0.8, head_length=0.4, fc='red', ec='red', alpha=0.8)

        # 速度方向
        c_psi, s_psi = np.cos(psi), np.sin(psi)
        vel_north = c_psi * u - s_psi * v
        vel_east = s_psi * u + c_psi * v
        vel_mag = np.sqrt(vel_north**2 + vel_east**2)
        if vel_mag > 0.1:
            vel_north_norm = vel_north / vel_mag * arrow_length
            vel_east_norm = vel_east / vel_mag * arrow_length
            ax.arrow(pe + 0.5, pn + 0.5, vel_east_norm, vel_north_norm,
                    head_width=0.8, head_length=0.4, fc='green', ec='green', alpha=0.8)

    ax.legend()

    # 右图：角度随时间变化
    ax = axes[1]
    ax.plot(t, np.rad2deg(states[:, 8]), 'b-', label='ψ (yaw)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Yaw Angle vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    print(f"\n数据检查：")
    print(f"初始 psi = {np.rad2deg(states[0, 8]):.1f}°")
    print(f"最终 psi = {np.rad2deg(states[-1, 8]):.1f}°")
    print(f"轨迹范围: North=[{states[:, 0].min():.1f}, {states[:, 0].max():.1f}]")
    print(f"轨迹范围: East=[{states[:, 1].min():.1f}, {states[:, 1].max():.1f}]")


def check_rotation_matrix():
    """检查旋转矩阵是否正确"""
    print("\n=== 检查旋转矩阵 ===")

    from mav_dynamics import MAVDynamics

    mav = MAVDynamics(1.0, 0.1, 0.1, 0.1, 0.0)

    # 测试不同偏航角
    for psi_deg in [0, 45, 90, 180]:
        psi = np.deg2rad(psi_deg)
        R = mav.rotation_matrix_body_to_inertial(0, 0, psi)

        # 机体系x轴（机头方向）在惯性系中的方向
        nose_dir = R @ np.array([1, 0, 0])

        # 机体系速度 [u, 0, 0] 在惯性系中的速度
        vel_inertial = R @ np.array([10, 0, 0])

        print(f"\npsi = {psi_deg}°:")
        print(f"  机头方向 (惯性系): North={nose_dir[0]:.3f}, East={nose_dir[1]:.3f}")
        print(f"  速度方向 (惯性系): North={vel_inertial[0]:.3f}, East={vel_inertial[1]:.3f}")

        # 检查是否一致
        if np.allclose(nose_dir[:2] / np.linalg.norm(nose_dir[:2]),
                       vel_inertial[:2] / np.linalg.norm(vel_inertial[:2])):
            print(f"  ✓ 机头和速度方向一致")
        else:
            print(f"  ✗ 机头和速度方向不一致！")


if __name__ == "__main__":
    check_rotation_matrix()
    debug_straight_flight()
    debug_yaw_rotation()
