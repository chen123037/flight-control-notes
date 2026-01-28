"""
第四章验证测试脚本
验证力与力矩模型的正确性

参考：设计要求4.3 - 通过设置控制面变化为不同的值来验证仿真
"""

import numpy as np
import matplotlib.pyplot as plt

from mav_params import ZagiParams
from mav_dynamics import StateIndex
from forces_moments import ForcesAndMoments, compute_trim_throttle
from mav_sim_ch4 import MAVSimulationCh4


def test_elevator_response():
    """测试升降舵响应"""
    print("\n=== 升降舵响应测试 ===")
    print("预期：升降舵向下偏(δe<0)产生抬头力矩，飞机俯仰角增加")

    sim = MAVSimulationCh4(ZagiParams, dt=0.01)

    # 初始状态：平飞
    Va = 25.0
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = Va
    initial_state[StateIndex.PD] = -100
    sim.set_initial_state(initial_state)

    delta_t = compute_trim_throttle(ZagiParams, Va)

    # 控制函数：在2-4秒施加升降舵
    def controls_func(t):
        delta_e = 0
        if 2 < t < 4:
            delta_e = np.deg2rad(-10)  # 升降舵向下偏转10度
        return [delta_e, 0, 0, delta_t]

    sim.set_wind(steady_wind_ned=None, turbulence_level=None)
    sim.simulate((0, 8), controls_func)

    # 绘图
    t = sim.time_history
    s = sim.state_history

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Elevator Response Test (δe = -10° during 2-4s)', fontsize=14)

    # 俯仰角
    ax = axes[0, 0]
    ax.plot(t, np.rad2deg(s[:, StateIndex.THETA]))
    ax.axvspan(2, 4, alpha=0.3, color='yellow', label='δe active')
    ax.set_ylabel('Pitch Angle θ (deg)')
    ax.set_title('Pitch Angle')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 俯仰角速度
    ax = axes[0, 1]
    ax.plot(t, np.rad2deg(s[:, StateIndex.Q]))
    ax.axvspan(2, 4, alpha=0.3, color='yellow')
    ax.set_ylabel('Pitch Rate q (deg/s)')
    ax.set_title('Pitch Rate')
    ax.grid(True, alpha=0.3)

    # 攻角
    ax = axes[1, 0]
    ax.plot(t, np.rad2deg(sim.alpha_history))
    ax.axvspan(2, 4, alpha=0.3, color='yellow')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle of Attack α (deg)')
    ax.set_title('Angle of Attack')
    ax.grid(True, alpha=0.3)

    # 高度
    ax = axes[1, 1]
    ax.plot(t, -s[:, StateIndex.PD])
    ax.axvspan(2, 4, alpha=0.3, color='yellow')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    print(f"初始俯仰角: {np.rad2deg(s[0, StateIndex.THETA]):.2f}°")
    print(f"最大俯仰角: {np.rad2deg(s[:, StateIndex.THETA].max()):.2f}°")


def test_aileron_response():
    """测试副翼响应"""
    print("\n=== 副翼响应测试 ===")
    print("预期：右副翼向上偏(δa>0)产生右滚力矩，飞机向右滚转")

    sim = MAVSimulationCh4(ZagiParams, dt=0.01)

    Va = 25.0
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = Va
    initial_state[StateIndex.PD] = -100
    sim.set_initial_state(initial_state)

    delta_t = compute_trim_throttle(ZagiParams, Va)

    # 控制函数：在2-3秒施加副翼
    def controls_func(t):
        delta_a = 0
        if 2 < t < 3:
            delta_a = np.deg2rad(10)  # 副翼偏转10度
        return [0, delta_a, 0, delta_t]

    sim.set_wind(steady_wind_ned=None, turbulence_level=None)
    sim.simulate((0, 8), controls_func)

    t = sim.time_history
    s = sim.state_history

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Aileron Response Test (δa = 10° during 2-3s)', fontsize=14)

    # 滚转角
    ax = axes[0, 0]
    ax.plot(t, np.rad2deg(s[:, StateIndex.PHI]))
    ax.axvspan(2, 3, alpha=0.3, color='yellow', label='δa active')
    ax.set_ylabel('Roll Angle φ (deg)')
    ax.set_title('Roll Angle')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 滚转角速度
    ax = axes[0, 1]
    ax.plot(t, np.rad2deg(s[:, StateIndex.P]))
    ax.axvspan(2, 3, alpha=0.3, color='yellow')
    ax.set_ylabel('Roll Rate p (deg/s)')
    ax.set_title('Roll Rate')
    ax.grid(True, alpha=0.3)

    # 侧滑角
    ax = axes[1, 0]
    ax.plot(t, np.rad2deg(sim.beta_history))
    ax.axvspan(2, 3, alpha=0.3, color='yellow')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Sideslip Angle β (deg)')
    ax.set_title('Sideslip Angle')
    ax.grid(True, alpha=0.3)

    # 偏航角
    ax = axes[1, 1]
    ax.plot(t, np.rad2deg(s[:, StateIndex.PSI]))
    ax.axvspan(2, 3, alpha=0.3, color='yellow')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw Angle ψ (deg)')
    ax.set_title('Yaw Angle')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    print(f"初始滚转角: {np.rad2deg(s[0, StateIndex.PHI]):.2f}°")
    print(f"最大滚转角: {np.rad2deg(s[:, StateIndex.PHI].max()):.2f}°")


def test_throttle_response():
    """测试油门响应"""
    print("\n=== 油门响应测试 ===")
    print("预期：增加油门会增加推力，飞机加速")

    sim = MAVSimulationCh4(ZagiParams, dt=0.01)

    Va = 20.0
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = Va
    initial_state[StateIndex.PD] = -100
    sim.set_initial_state(initial_state)

    # 控制函数：在2-4秒增加油门
    def controls_func(t):
        if t < 2:
            delta_t = 0.3
        elif t < 4:
            delta_t = 0.8  # 增加油门
        else:
            delta_t = 0.3
        return [0, 0, 0, delta_t]

    sim.set_wind(steady_wind_ned=None, turbulence_level=None)
    sim.simulate((0, 10), controls_func)

    t = sim.time_history
    s = sim.state_history

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Throttle Response Test (δt = 0.8 during 2-4s)', fontsize=14)

    # 空速
    ax = axes[0, 0]
    ax.plot(t, sim.Va_history)
    ax.axvspan(2, 4, alpha=0.3, color='yellow', label='High throttle')
    ax.set_ylabel('Airspeed Va (m/s)')
    ax.set_title('Airspeed')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 前向速度
    ax = axes[0, 1]
    ax.plot(t, s[:, StateIndex.U])
    ax.axvspan(2, 4, alpha=0.3, color='yellow')
    ax.set_ylabel('Forward Velocity u (m/s)')
    ax.set_title('Forward Velocity')
    ax.grid(True, alpha=0.3)

    # 高度
    ax = axes[1, 0]
    ax.plot(t, -s[:, StateIndex.PD])
    ax.axvspan(2, 4, alpha=0.3, color='yellow')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude')
    ax.grid(True, alpha=0.3)

    # X方向力
    ax = axes[1, 1]
    ax.plot(t, sim.forces_history[:, 0])
    ax.axvspan(2, 4, alpha=0.3, color='yellow')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('fx (N)')
    ax.set_title('Forward Force')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    print(f"初始空速: {sim.Va_history[0]:.2f} m/s")
    print(f"最大空速: {sim.Va_history.max():.2f} m/s")


def test_wind_effect():
    """测试风的影响"""
    print("\n=== 风影响测试 ===")
    print("对比：无风 vs 有风（北风5m/s + 干扰风）")

    Va = 25.0
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = Va
    initial_state[StateIndex.PD] = -100

    delta_t = compute_trim_throttle(ZagiParams, Va)
    controls = [np.deg2rad(-2), 0, 0, delta_t]

    # 无风仿真
    sim_no_wind = MAVSimulationCh4(ZagiParams, dt=0.01)
    sim_no_wind.set_initial_state(initial_state.copy())
    sim_no_wind.set_controls(*controls)
    sim_no_wind.set_wind(steady_wind_ned=None, turbulence_level=None)
    sim_no_wind.simulate((0, 15))

    # 有风仿真
    sim_with_wind = MAVSimulationCh4(ZagiParams, dt=0.01)
    sim_with_wind.set_initial_state(initial_state.copy())
    sim_with_wind.set_controls(*controls)
    sim_with_wind.set_wind(
        steady_wind_ned=[5, 0, 0],  # 北风5m/s
        turbulence_level='light_low',
        Va=Va
    )
    sim_with_wind.simulate((0, 15))

    # 绘图
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('Wind Effect Comparison', fontsize=14)

    t1 = sim_no_wind.time_history
    s1 = sim_no_wind.state_history
    t2 = sim_with_wind.time_history
    s2 = sim_with_wind.state_history

    # 地面轨迹
    ax = axes[0, 0]
    ax.plot(s1[:, StateIndex.PE], s1[:, StateIndex.PN], 'b-', label='No wind')
    ax.plot(s2[:, StateIndex.PE], s2[:, StateIndex.PN], 'r-', label='With wind')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Ground Track')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

    # 空速
    ax = axes[0, 1]
    ax.plot(t1, sim_no_wind.Va_history, 'b-', label='No wind')
    ax.plot(t2, sim_with_wind.Va_history, 'r-', label='With wind')
    ax.set_ylabel('Airspeed Va (m/s)')
    ax.set_title('Airspeed')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 高度
    ax = axes[0, 2]
    ax.plot(t1, -s1[:, StateIndex.PD], 'b-', label='No wind')
    ax.plot(t2, -s2[:, StateIndex.PD], 'r-', label='With wind')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 攻角
    ax = axes[1, 0]
    ax.plot(t1, np.rad2deg(sim_no_wind.alpha_history), 'b-', label='No wind')
    ax.plot(t2, np.rad2deg(sim_with_wind.alpha_history), 'r-', label='With wind')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle of Attack α (deg)')
    ax.set_title('Angle of Attack')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 俯仰角
    ax = axes[1, 1]
    ax.plot(t1, np.rad2deg(s1[:, StateIndex.THETA]), 'b-', label='No wind')
    ax.plot(t2, np.rad2deg(s2[:, StateIndex.THETA]), 'r-', label='With wind')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch Angle θ (deg)')
    ax.set_title('Pitch Angle')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 风速
    ax = axes[1, 2]
    ax.plot(t2, sim_with_wind.wind_history[:, 0], label='wn')
    ax.plot(t2, sim_with_wind.wind_history[:, 1], label='we')
    ax.plot(t2, sim_with_wind.wind_history[:, 2], label='wd')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Wind (m/s)')
    ax.set_title('Wind (NED)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    print(f"\n无风最终位置: North={s1[-1, 0]:.1f}m, East={s1[-1, 1]:.1f}m")
    print(f"有风最终位置: North={s2[-1, 0]:.1f}m, East={s2[-1, 1]:.1f}m")


def run_interactive_menu():
    """交互式菜单"""
    while True:
        print("\n" + "=" * 50)
        print("第四章验证测试菜单")
        print("=" * 50)
        print("1. 升降舵响应测试")
        print("2. 副翼响应测试")
        print("3. 油门响应测试")
        print("4. 风影响对比测试")
        print("5. 运行完整仿真（有交互菜单）")
        print("0. 退出")
        print("-" * 50)

        try:
            choice = input("请选择 (0-5): ").strip()
            if choice == "0":
                break
            elif choice == "1":
                test_elevator_response()
            elif choice == "2":
                test_aileron_response()
            elif choice == "3":
                test_throttle_response()
            elif choice == "4":
                test_wind_effect()
            elif choice == "5":
                import mav_sim_ch4
                mav_sim_ch4.demo_level_flight()
            else:
                print("无效选择")
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    run_interactive_menu()
