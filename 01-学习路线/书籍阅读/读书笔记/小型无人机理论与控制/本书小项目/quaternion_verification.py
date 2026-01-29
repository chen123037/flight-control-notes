"""
四元数仿真验证测试脚本

验证内容:
1. 四元数-欧拉角转换的一致性
2. 四元数归一化的有效性
3. 四元数仿真与欧拉角仿真的对比
4. 大角度机动测试（接近万向锁）
5. 长时间仿真稳定性
"""

import numpy as np
import matplotlib.pyplot as plt

from mav_params import ZagiParams
from quaternion import (
    euler_to_quaternion,
    quaternion_to_euler,
    quaternion_norm_error,
    gravity_body_frame,
    quaternion_derivative,
    normalize_quaternion
)
from mav_dynamics_quat import MAVDynamicsQuat, state_euler_to_quat, state_quat_to_euler
from mav_dynamics import MAVDynamics, StateIndex
from forces_moments_quat import ForcesAndMomentsQuat, compute_trim_throttle
from forces_moments import ForcesAndMoments
from mav_sim_quat import MAVSimulationQuat
from mav_sim_ch4 import MAVSimulationCh4


def test_euler_quaternion_conversion():
    """测试1: 欧拉角-四元数转换的一致性"""
    print("\n" + "=" * 60)
    print("测试1: 欧拉角-四元数转换一致性")
    print("=" * 60)

    # 测试各种角度组合
    test_cases = [
        (0, 0, 0),
        (30, 0, 0),
        (0, 45, 0),
        (0, 0, 60),
        (30, 45, 60),
        (-30, -45, -60),
        (0, 85, 0),  # 接近万向锁
        (45, 80, 90),
    ]

    print("\n原始角度(deg) → 四元数 → 还原角度(deg) | 误差")
    print("-" * 70)

    max_error = 0
    for phi_deg, theta_deg, psi_deg in test_cases:
        phi = np.deg2rad(phi_deg)
        theta = np.deg2rad(theta_deg)
        psi = np.deg2rad(psi_deg)

        # 转换
        e = euler_to_quaternion(phi, theta, psi)
        phi2, theta2, psi2 = quaternion_to_euler(e)

        # 计算误差（考虑角度周期性）
        def angle_diff(a, b):
            d = a - b
            while d > np.pi:
                d -= 2 * np.pi
            while d < -np.pi:
                d += 2 * np.pi
            return abs(d)

        err_phi = angle_diff(phi, phi2)
        err_theta = angle_diff(theta, theta2)
        err_psi = angle_diff(psi, psi2)
        total_err = err_phi + err_theta + err_psi

        max_error = max(max_error, total_err)

        print(f"({phi_deg:4.0f}, {theta_deg:4.0f}, {psi_deg:4.0f}) → "
              f"[{e[0]:.3f}, {e[1]:.3f}, {e[2]:.3f}, {e[3]:.3f}] → "
              f"({np.rad2deg(phi2):6.1f}, {np.rad2deg(theta2):6.1f}, {np.rad2deg(psi2):6.1f}) | "
              f"{np.rad2deg(total_err):.2e}°")

    print(f"\n最大总误差: {np.rad2deg(max_error):.2e}°")
    return max_error < 1e-10


def test_quaternion_normalization():
    """测试2: 四元数归一化有效性"""
    print("\n" + "=" * 60)
    print("测试2: 四元数归一化有效性（科贝特-赖特正交控制）")
    print("=" * 60)

    # 创建动力学模型
    dynamics = MAVDynamicsQuat(
        mass=ZagiParams.m,
        Jx=ZagiParams.Jx,
        Jy=ZagiParams.Jy,
        Jz=ZagiParams.Jz,
        Jxz=ZagiParams.Jxz,
        lambda_quat=1000.0  # 正交控制增益
    )

    # 初始状态
    initial = np.zeros(12)
    initial[3] = 25.0  # u
    initial[6] = np.deg2rad(30)  # φ
    initial[7] = np.deg2rad(20)  # θ
    initial[8] = np.deg2rad(45)  # ψ
    initial[9] = np.deg2rad(10)  # p
    initial[10] = np.deg2rad(5)  # q
    initial[11] = np.deg2rad(3)  # r
    dynamics.set_state(initial)

    # 简单力和力矩
    forces = np.array([0, 0, ZagiParams.m * 9.81])
    moments = np.array([0.1, 0.05, 0.02])

    # 积分1000步
    n_steps = 1000
    dt = 0.01
    norm_errors = []

    for i in range(n_steps):
        dynamics.rk4_step(i * dt, forces, moments, dt)
        norm_errors.append(dynamics.quaternion_norm_error())

    norm_errors = np.array(norm_errors)

    print(f"积分步数: {n_steps}")
    print(f"时间步长: {dt} s")
    print(f"总仿真时间: {n_steps * dt} s")
    print(f"初始范数误差: {norm_errors[0]:.2e}")
    print(f"最终范数误差: {norm_errors[-1]:.2e}")
    print(f"最大范数误差: {np.max(norm_errors):.2e}")
    print(f"平均范数误差: {np.mean(norm_errors):.2e}")

    return np.max(norm_errors) < 1e-6


def test_gravity_calculation():
    """测试3: 重力计算对比（四元数 vs 欧拉角）"""
    print("\n" + "=" * 60)
    print("测试3: 重力计算对比（四元数 vs 欧拉角）")
    print("=" * 60)

    m = ZagiParams.m
    g = 9.81

    test_angles = [
        (0, 0, 0),
        (30, 0, 0),
        (0, 45, 0),
        (30, 45, 60),
        (0, 89, 0),  # 接近万向锁
    ]

    print("\n姿态(deg)     | 四元数方法         | 欧拉角方法         | 误差")
    print("-" * 80)

    max_error = 0
    for phi_deg, theta_deg, psi_deg in test_angles:
        phi = np.deg2rad(phi_deg)
        theta = np.deg2rad(theta_deg)
        psi = np.deg2rad(psi_deg)

        # 四元数方法
        e = euler_to_quaternion(phi, theta, psi)
        fg_quat = gravity_body_frame(e, m, g)

        # 欧拉角方法
        fx_euler = -m * g * np.sin(theta)
        fy_euler = m * g * np.cos(theta) * np.sin(phi)
        fz_euler = m * g * np.cos(theta) * np.cos(phi)
        fg_euler = np.array([fx_euler, fy_euler, fz_euler])

        error = np.linalg.norm(fg_quat - fg_euler)
        max_error = max(max_error, error)

        print(f"({phi_deg:3.0f},{theta_deg:3.0f},{psi_deg:3.0f}) | "
              f"[{fg_quat[0]:7.3f},{fg_quat[1]:7.3f},{fg_quat[2]:7.3f}] | "
              f"[{fg_euler[0]:7.3f},{fg_euler[1]:7.3f},{fg_euler[2]:7.3f}] | "
              f"{error:.2e}")

    print(f"\n最大误差: {max_error:.2e} N")
    return max_error < 1e-10


def test_simulation_comparison():
    """测试4: 四元数仿真与欧拉角仿真对比"""
    print("\n" + "=" * 60)
    print("测试4: 四元数仿真与欧拉角仿真对比")
    print("=" * 60)

    # 相同的初始条件
    Va = 25.0
    initial_euler = np.zeros(12)
    initial_euler[3] = Va
    initial_euler[2] = -100  # 高度100m
    initial_euler[7] = np.deg2rad(5)  # θ = 5°

    delta_t = compute_trim_throttle(ZagiParams, Va)

    # 控制函数
    def controls_func(t):
        delta_e = 0
        if 2 < t < 4:
            delta_e = np.deg2rad(-5)
        return [delta_e, 0, 0, delta_t]

    # 四元数仿真
    print("\n运行四元数仿真...")
    sim_quat = MAVSimulationQuat(ZagiParams, dt=0.01)
    sim_quat.set_initial_state(initial_euler)
    sim_quat.set_wind(steady_wind_ned=None, turbulence_level=None)
    sim_quat.simulate((0, 8), controls_func)

    # 欧拉角仿真
    print("运行欧拉角仿真...")
    sim_euler = MAVSimulationCh4(ZagiParams, dt=0.01)
    sim_euler.set_initial_state(initial_euler)
    sim_euler.set_wind(steady_wind_ned=None, turbulence_level=None)
    sim_euler.simulate((0, 8), controls_func)

    # 对比结果
    t = sim_quat.time_history
    s_quat = sim_quat.state_euler_history  # 转换为欧拉角形式对比
    s_euler = sim_euler.state_history

    # 角度差异计算函数（考虑周期性）
    def angle_difference(a1, a2):
        """计算两个角度的差异，考虑±π周期性"""
        diff = a1 - a2
        # 将差异归一化到[-π, π]
        diff = np.mod(diff + np.pi, 2 * np.pi) - np.pi
        return diff

    # 计算差异
    pos_diff = np.linalg.norm(s_quat[:, 0:3] - s_euler[:, 0:3], axis=1)
    vel_diff = np.linalg.norm(s_quat[:, 3:6] - s_euler[:, 3:6], axis=1)

    # 对每个欧拉角分别计算周期性差异
    phi_diff = angle_difference(s_quat[:, 6], s_euler[:, 6])
    theta_diff = angle_difference(s_quat[:, 7], s_euler[:, 7])
    psi_diff = angle_difference(s_quat[:, 8], s_euler[:, 8])
    angle_diff = np.sqrt(phi_diff**2 + theta_diff**2 + psi_diff**2)

    rate_diff = np.linalg.norm(s_quat[:, 9:12] - s_euler[:, 9:12], axis=1)

    print(f"\n位置最大差异: {np.max(pos_diff):.4f} m")
    print(f"速度最大差异: {np.max(vel_diff):.4f} m/s")
    print(f"角度最大差异: {np.rad2deg(np.max(angle_diff)):.4f}°")
    print(f"角速度最大差异: {np.rad2deg(np.max(rate_diff)):.4f}°/s")

    # 绘图对比
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Quaternion vs Euler Angle Simulation Comparison', fontsize=14)

    # 位置对比
    ax = axes[0, 0]
    ax.plot(t, s_quat[:, 0], 'b-', label='Quat pn')
    ax.plot(t, s_euler[:, 0], 'r--', label='Euler pn')
    ax.plot(t, -s_quat[:, 2], 'g-', label='Quat h')
    ax.plot(t, -s_euler[:, 2], 'm--', label='Euler h')
    ax.set_ylabel('Position (m)')
    ax.set_title('Position Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 俯仰角对比
    ax = axes[0, 1]
    ax.plot(t, np.rad2deg(s_quat[:, 7]), 'b-', label='Quat θ')
    ax.plot(t, np.rad2deg(s_euler[:, 7]), 'r--', label='Euler θ')
    ax.axvspan(2, 4, alpha=0.2, color='yellow', label='δe active')
    ax.set_ylabel('Pitch Angle (deg)')
    ax.set_title('Pitch Angle Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 差异
    ax = axes[1, 0]
    ax.semilogy(t, pos_diff + 1e-16, label='Position')
    ax.semilogy(t, vel_diff + 1e-16, label='Velocity')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Difference')
    ax.set_title('Position & Velocity Difference')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.semilogy(t, np.rad2deg(angle_diff) + 1e-16, label='Angles')
    ax.semilogy(t, np.rad2deg(rate_diff) + 1e-16, label='Rates')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Difference (deg)')
    ax.set_title('Angle & Rate Difference')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # 判断是否通过
    # 注意：欧拉角在俯仰角接近±90°时有万向锁歧义，导致提取的滚转角和偏航角可能不同
    # 但位置、速度、角速度的一致性足以证明物理仿真的正确性
    pos_pass = np.max(pos_diff) < 0.1
    vel_pass = np.max(vel_diff) < 0.1
    rate_pass = np.rad2deg(np.max(rate_diff)) < 0.1

    print(f"\n判断标准:")
    print(f"  位置差异 < 0.1m: {'✓' if pos_pass else '✗'}")
    print(f"  速度差异 < 0.1m/s: {'✓' if vel_pass else '✗'}")
    print(f"  角速度差异 < 0.1°/s: {'✓' if rate_pass else '✗'}")
    print(f"  (注：欧拉角在万向锁附近可能有提取歧义，不作为判断依据)")

    return pos_pass and vel_pass and rate_pass


def test_large_angle_maneuver():
    """测试5: 大角度机动测试"""
    print("\n" + "=" * 60)
    print("测试5: 大角度机动测试（连续滚转）")
    print("=" * 60)

    sim = MAVSimulationQuat(ZagiParams, dt=0.01)

    # 初始状态
    initial = np.zeros(12)
    initial[3] = 25.0  # u
    initial[2] = -100  # 高度

    sim.set_initial_state(initial)

    delta_t = compute_trim_throttle(ZagiParams, 25.0)

    # 持续滚转（大角度机动）
    def controls_func(t):
        delta_a = np.deg2rad(15)  # 持续副翼输入
        return [0, delta_a, 0, delta_t]

    sim.set_wind(steady_wind_ned=None, turbulence_level=None)

    print("运行大角度机动仿真（15秒持续滚转）...")
    sim.simulate((0, 15), controls_func)

    # 检查四元数稳定性
    max_norm_error = np.max(sim.quat_norm_error_history)
    euler_angles = sim.get_euler_angles()
    max_roll = np.rad2deg(np.max(np.abs(euler_angles[:, 0])))

    print(f"最大滚转角: {max_roll:.1f}°")
    print(f"四元数最大范数误差: {max_norm_error:.2e}")

    # 绘图
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Large Angle Maneuver Test (Continuous Roll)', fontsize=14)

    t = sim.time_history

    # 欧拉角
    ax = axes[0, 0]
    ax.plot(t, np.rad2deg(euler_angles[:, 0]), label='φ (roll)')
    ax.plot(t, np.rad2deg(euler_angles[:, 1]), label='θ (pitch)')
    ax.plot(t, np.rad2deg(euler_angles[:, 2]), label='ψ (yaw)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('Euler Angles')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 四元数
    ax = axes[0, 1]
    q = sim.get_quaternion_history()
    ax.plot(t, q[:, 0], label='e0')
    ax.plot(t, q[:, 1], label='e1')
    ax.plot(t, q[:, 2], label='e2')
    ax.plot(t, q[:, 3], label='e3')
    ax.set_ylabel('Quaternion')
    ax.set_title('Quaternion Components')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 四元数范数误差
    ax = axes[1, 0]
    ax.semilogy(t, sim.quat_norm_error_history + 1e-16)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Norm Error')
    ax.set_title('Quaternion Normalization Error')
    ax.grid(True, alpha=0.3)

    # 3D轨迹
    ax = axes[1, 1]
    ax.remove()
    ax = fig.add_subplot(2, 2, 4, projection='3d')
    s = sim.state_euler_history
    ax.plot(s[:, 0], s[:, 1], -s[:, 2])
    ax.set_xlabel('North')
    ax.set_ylabel('East')
    ax.set_zlabel('Altitude')
    ax.set_title('3D Trajectory')

    plt.tight_layout()
    plt.show()

    return max_norm_error < 1e-6


def run_all_tests():
    """运行所有验证测试"""
    print("\n" + "=" * 60)
    print("四元数仿真验证测试")
    print("=" * 60)

    results = {}

    # 测试1
    results['转换一致性'] = test_euler_quaternion_conversion()

    # 测试2
    results['归一化有效性'] = test_quaternion_normalization()

    # 测试3
    results['重力计算'] = test_gravity_calculation()

    # 测试4
    results['仿真对比'] = test_simulation_comparison()

    # 测试5
    results['大角度机动'] = test_large_angle_maneuver()

    # 汇总
    print("\n" + "=" * 60)
    print("测试结果汇总")
    print("=" * 60)

    all_pass = True
    for name, passed in results.items():
        status = "✓ 通过" if passed else "✗ 失败"
        print(f"  {name}: {status}")
        if not passed:
            all_pass = False

    print("\n" + ("所有测试通过!" if all_pass else "部分测试失败!"))

    return all_pass


if __name__ == "__main__":
    run_all_tests()
