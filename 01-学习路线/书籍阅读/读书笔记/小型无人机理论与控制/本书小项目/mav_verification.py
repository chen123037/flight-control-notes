"""
MAV动力学模型验证测试
参考：第三章设计要求 3.3 和 3.4

验证内容：
1. 单独施加力验证（fx, fy, fz）
2. 单独施加力矩验证（l, m, n）
3. 滚转-偏航耦合验证（Jxz的影响）
"""

import numpy as np
import matplotlib.pyplot as plt

from mav_params import ZagiParams, g
from mav_dynamics import MAVDynamics, create_mav_from_params, StateIndex


def test_single_force(force_name, force_value, duration=3.0):
    """
    测试单个力的响应

    参数:
        force_name: 力的名称 ('fx', 'fy', 'fz')
        force_value: 力的大小 (N)
        duration: 仿真时长 (s)
    """
    print(f"\n=== 测试 {force_name} = {force_value} N ===")

    mav = create_mav_from_params(ZagiParams)

    # 初始状态：静止
    initial_state = np.zeros(12)
    initial_state[StateIndex.PD] = -100.0  # 高度100m
    mav.set_state(initial_state)

    force_idx = {"fx": 0, "fy": 1, "fz": 2}[force_name]

    def forces(t, state):
        f = np.zeros(3)
        f[force_idx] = force_value
        return f

    def moments(t, state):
        return np.zeros(3)

    t, states = mav.simulate((0, duration), forces, moments, dt=0.01)

    # 绘制结果
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f"Force Test: {force_name} = {force_value} N", fontsize=14)

    # 位置
    ax = axes[0, 0]
    ax.plot(t, states[:, 0], label="pn (North)")
    ax.plot(t, states[:, 1], label="pe (East)")
    ax.plot(t, -states[:, 2], label="h (Altitude)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Position")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 速度
    ax = axes[0, 1]
    ax.plot(t, states[:, 3], label="u")
    ax.plot(t, states[:, 4], label="v")
    ax.plot(t, states[:, 5], label="w")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Body Velocities")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 欧拉角
    ax = axes[1, 0]
    ax.plot(t, np.rad2deg(states[:, 6]), label="φ (roll)")
    ax.plot(t, np.rad2deg(states[:, 7]), label="θ (pitch)")
    ax.plot(t, np.rad2deg(states[:, 8]), label="ψ (yaw)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Euler Angles")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 角速度
    ax = axes[1, 1]
    ax.plot(t, np.rad2deg(states[:, 9]), label="p")
    ax.plot(t, np.rad2deg(states[:, 10]), label="q")
    ax.plot(t, np.rad2deg(states[:, 11]), label="r")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Rate (deg/s)")
    ax.set_title("Angular Velocities")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # 打印关键结果
    print(f"  最终位置: pn={states[-1, 0]:.2f}m, pe={states[-1, 1]:.2f}m, h={-states[-1, 2]:.2f}m")
    print(f"  最终速度: u={states[-1, 3]:.2f}m/s, v={states[-1, 4]:.2f}m/s, w={states[-1, 5]:.2f}m/s")


def test_single_moment(moment_name, moment_value, duration=3.0):
    """
    测试单个力矩的响应

    参数:
        moment_name: 力矩的名称 ('l', 'm', 'n')
        moment_value: 力矩的大小 (N·m)
        duration: 仿真时长 (s)
    """
    print(f"\n=== 测试 {moment_name} = {moment_value} N·m ===")

    mav = create_mav_from_params(ZagiParams)

    # 初始状态：向前飞行
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 20.0
    initial_state[StateIndex.PD] = -100.0
    mav.set_state(initial_state)

    moment_idx = {"l": 0, "m": 1, "n": 2}[moment_name]

    def forces(t, state):
        # 包含重力
        phi = state[StateIndex.PHI]
        theta = state[StateIndex.THETA]
        return mav.get_gravity_force_body(phi, theta)

    def moments(t, state):
        m = np.zeros(3)
        m[moment_idx] = moment_value
        return m

    t, states = mav.simulate((0, duration), forces, moments, dt=0.01)

    # 绘制结果
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f"Moment Test: {moment_name} = {moment_value} N·m", fontsize=14)

    # 位置
    ax = axes[0, 0]
    ax.plot(t, states[:, 0], label="pn (North)")
    ax.plot(t, states[:, 1], label="pe (East)")
    ax.plot(t, -states[:, 2], label="h (Altitude)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Position")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 速度
    ax = axes[0, 1]
    ax.plot(t, states[:, 3], label="u")
    ax.plot(t, states[:, 4], label="v")
    ax.plot(t, states[:, 5], label="w")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Body Velocities")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 欧拉角
    ax = axes[1, 0]
    ax.plot(t, np.rad2deg(states[:, 6]), label="φ (roll)")
    ax.plot(t, np.rad2deg(states[:, 7]), label="θ (pitch)")
    ax.plot(t, np.rad2deg(states[:, 8]), label="ψ (yaw)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Euler Angles")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 角速度
    ax = axes[1, 1]
    ax.plot(t, np.rad2deg(states[:, 9]), label="p (roll rate)")
    ax.plot(t, np.rad2deg(states[:, 10]), label="q (pitch rate)")
    ax.plot(t, np.rad2deg(states[:, 11]), label="r (yaw rate)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Rate (deg/s)")
    ax.set_title("Angular Velocities")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # 打印关键结果
    print(f"  最终欧拉角: φ={np.rad2deg(states[-1, 6]):.2f}°, θ={np.rad2deg(states[-1, 7]):.2f}°, ψ={np.rad2deg(states[-1, 8]):.2f}°")
    print(f"  最终角速度: p={np.rad2deg(states[-1, 9]):.2f}°/s, q={np.rad2deg(states[-1, 10]):.2f}°/s, r={np.rad2deg(states[-1, 11]):.2f}°/s")


def test_roll_yaw_coupling():
    """
    测试滚转-偏航耦合（设计要求3.4）

    验证：
    1. Jxz = 0 时，滚转力矩l只产生滚转，偏航力矩n只产生偏航
    2. Jxz ≠ 0 时，滚转和偏航之间存在耦合
    """
    print("\n" + "=" * 60)
    print("滚转-偏航耦合验证测试")
    print("=" * 60)

    duration = 2.0
    moment_value = 0.5  # N·m

    # --- 测试1：Jxz = 0，施加滚转力矩 ---
    print("\n--- 测试1: Jxz = 0, 施加滚转力矩 l ---")
    mav_no_coupling = MAVDynamics(
        mass=ZagiParams.m,
        Jx=ZagiParams.Jx,
        Jy=ZagiParams.Jy,
        Jz=ZagiParams.Jz,
        Jxz=0.0  # 设为0
    )

    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 20.0
    initial_state[StateIndex.PD] = -100.0
    mav_no_coupling.set_state(initial_state)

    def forces_gravity(t, state):
        phi = state[StateIndex.PHI]
        theta = state[StateIndex.THETA]
        return mav_no_coupling.get_gravity_force_body(phi, theta)

    def moments_roll(t, state):
        return np.array([moment_value, 0.0, 0.0])  # 只有滚转力矩

    t1, states1 = mav_no_coupling.simulate((0, duration), forces_gravity, moments_roll, dt=0.01)

    # --- 测试2：Jxz ≠ 0，施加滚转力矩 ---
    print("--- 测试2: Jxz ≠ 0, 施加滚转力矩 l ---")
    mav_with_coupling = create_mav_from_params(ZagiParams)

    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 20.0
    initial_state[StateIndex.PD] = -100.0
    mav_with_coupling.set_state(initial_state)

    def forces_gravity2(t, state):
        phi = state[StateIndex.PHI]
        theta = state[StateIndex.THETA]
        return mav_with_coupling.get_gravity_force_body(phi, theta)

    t2, states2 = mav_with_coupling.simulate((0, duration), forces_gravity2, moments_roll, dt=0.01)

    # --- 测试3：Jxz = 0，施加偏航力矩 ---
    print("--- 测试3: Jxz = 0, 施加偏航力矩 n ---")
    mav_no_coupling.set_state(initial_state)

    def moments_yaw(t, state):
        return np.array([0.0, 0.0, moment_value])  # 只有偏航力矩

    t3, states3 = mav_no_coupling.simulate((0, duration), forces_gravity, moments_yaw, dt=0.01)

    # --- 测试4：Jxz ≠ 0，施加偏航力矩 ---
    print("--- 测试4: Jxz ≠ 0, 施加偏航力矩 n ---")
    mav_with_coupling.set_state(initial_state)
    t4, states4 = mav_with_coupling.simulate((0, duration), forces_gravity2, moments_yaw, dt=0.01)

    # --- 绘制对比图 ---
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Roll-Yaw Coupling Verification (Jxz Effect)", fontsize=14)

    # 图1：滚转力矩 -> 滚转角速度p
    ax = axes[0, 0]
    ax.plot(t1, np.rad2deg(states1[:, 9]), "b-", label="Jxz=0 (p)")
    ax.plot(t2, np.rad2deg(states2[:, 9]), "r--", label="Jxz≠0 (p)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Roll Rate p (deg/s)")
    ax.set_title("Roll Moment → Roll Rate")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 图2：滚转力矩 -> 偏航角速度r
    ax = axes[0, 1]
    ax.plot(t1, np.rad2deg(states1[:, 11]), "b-", label="Jxz=0 (r)")
    ax.plot(t2, np.rad2deg(states2[:, 11]), "r--", label="Jxz≠0 (r)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw Rate r (deg/s)")
    ax.set_title("Roll Moment → Yaw Rate (Coupling)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 图3：偏航力矩 -> 偏航角速度r
    ax = axes[1, 0]
    ax.plot(t3, np.rad2deg(states3[:, 11]), "b-", label="Jxz=0 (r)")
    ax.plot(t4, np.rad2deg(states4[:, 11]), "r--", label="Jxz≠0 (r)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw Rate r (deg/s)")
    ax.set_title("Yaw Moment → Yaw Rate")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 图4：偏航力矩 -> 滚转角速度p
    ax = axes[1, 1]
    ax.plot(t3, np.rad2deg(states3[:, 9]), "b-", label="Jxz=0 (p)")
    ax.plot(t4, np.rad2deg(states4[:, 9]), "r--", label="Jxz≠0 (p)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Roll Rate p (deg/s)")
    ax.set_title("Yaw Moment → Roll Rate (Coupling)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # 打印验证结论
    print("\n" + "=" * 60)
    print("验证结论:")
    print("=" * 60)

    # 检查滚转力矩是否产生偏航
    r_from_l_no_coupling = np.max(np.abs(states1[:, 11]))
    r_from_l_with_coupling = np.max(np.abs(states2[:, 11]))
    print(f"\n滚转力矩l产生的偏航角速度r:")
    print(f"  Jxz=0:  最大|r| = {np.rad2deg(r_from_l_no_coupling):.4f} °/s")
    print(f"  Jxz≠0: 最大|r| = {np.rad2deg(r_from_l_with_coupling):.4f} °/s")

    if r_from_l_no_coupling < 0.001 and r_from_l_with_coupling > 0.001:
        print("  ✓ 验证通过：Jxz=0时无耦合，Jxz≠0时存在耦合")
    else:
        print("  ✗ 需要检查")

    # 检查偏航力矩是否产生滚转
    p_from_n_no_coupling = np.max(np.abs(states3[:, 9]))
    p_from_n_with_coupling = np.max(np.abs(states4[:, 9]))
    print(f"\n偏航力矩n产生的滚转角速度p:")
    print(f"  Jxz=0:  最大|p| = {np.rad2deg(p_from_n_no_coupling):.4f} °/s")
    print(f"  Jxz≠0: 最大|p| = {np.rad2deg(p_from_n_with_coupling):.4f} °/s")

    if p_from_n_no_coupling < 0.001 and p_from_n_with_coupling > 0.001:
        print("  ✓ 验证通过：Jxz=0时无耦合，Jxz≠0时存在耦合")
    else:
        print("  ✗ 需要检查")


def run_all_tests():
    """运行所有验证测试"""
    print("=" * 60)
    print("MAV动力学模型验证测试")
    print("=" * 60)

    # 测试力响应
    print("\n[1] 力响应测试")
    test_single_force("fx", 5.0, duration=2.0)   # 前向力
    test_single_force("fy", 5.0, duration=2.0)   # 侧向力
    test_single_force("fz", -5.0, duration=2.0)  # 垂向力（向上）

    # 测试力矩响应
    print("\n[2] 力矩响应测试")
    test_single_moment("l", 0.5, duration=2.0)  # 滚转力矩
    test_single_moment("m", 0.5, duration=2.0)  # 俯仰力矩
    test_single_moment("n", 0.5, duration=2.0)  # 偏航力矩

    # 测试滚转-偏航耦合
    print("\n[3] 滚转-偏航耦合测试")
    test_roll_yaw_coupling()


def run_interactive_menu():
    """交互式菜单"""
    while True:
        print("\n" + "=" * 50)
        print("MAV动力学验证测试菜单")
        print("=" * 50)
        print("1. 测试前向力 fx")
        print("2. 测试侧向力 fy")
        print("3. 测试垂向力 fz")
        print("4. 测试滚转力矩 l")
        print("5. 测试俯仰力矩 m")
        print("6. 测试偏航力矩 n")
        print("7. 滚转-偏航耦合验证 (Jxz)")
        print("8. 运行所有测试")
        print("0. 退出")
        print("-" * 50)

        try:
            choice = input("请选择测试项 (0-8): ").strip()

            if choice == "0":
                print("退出测试程序")
                break
            elif choice == "1":
                test_single_force("fx", 5.0)
            elif choice == "2":
                test_single_force("fy", 5.0)
            elif choice == "3":
                test_single_force("fz", -5.0)
            elif choice == "4":
                test_single_moment("l", 0.5)
            elif choice == "5":
                test_single_moment("m", 0.5)
            elif choice == "6":
                test_single_moment("n", 0.5)
            elif choice == "7":
                test_roll_yaw_coupling()
            elif choice == "8":
                run_all_tests()
            else:
                print("无效选择，请重新输入")
        except KeyboardInterrupt:
            print("\n退出测试程序")
            break


if __name__ == "__main__":
    # 默认运行交互式菜单
    # 如果想直接运行所有测试，可以调用 run_all_tests()
    run_interactive_menu()
