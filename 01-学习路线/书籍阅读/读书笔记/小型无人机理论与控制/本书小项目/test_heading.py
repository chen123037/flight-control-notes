"""
测试机头朝向是否正确
让飞机做一个偏航转弯，观察机头是否指向运动方向
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from mav_params import ZagiParams
from mav_dynamics import create_mav_from_params, StateIndex
from mav_simulation import MAVSimulation


def test_yaw_turn():
    """测试偏航转弯 - 验证机头朝向"""
    print("=== 测试：偏航转弯（验证机头朝向）===")
    print("飞机将向右转弯，观察机头是否始终指向运动方向")

    mav = create_mav_from_params(ZagiParams)

    # 初始状态：向北飞行
    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 15.0      # 前向速度 15 m/s
    initial_state[StateIndex.PD] = -50.0    # 高度 50m
    mav.set_state(initial_state)

    # 力函数：重力 + 一个小的向前推力（维持速度）
    def forces(t, state):
        phi = state[StateIndex.PHI]
        theta = state[StateIndex.THETA]
        # 重力
        fg = mav.get_gravity_force_body(phi, theta)
        # 推力（简单补偿重力的前向分量）
        thrust = np.array([2.0, 0.0, 0.0])
        return fg + thrust

    # 力矩函数：施加一个恒定的偏航力矩，让飞机右转
    def moments(t, state):
        # 偏航力矩 n > 0 -> 向右偏航
        return np.array([0.0, 0.0, 0.3])

    # 创建仿真
    sim = MAVSimulation(mav, viewer_scale=3.0)

    # 运行仿真 8 秒
    sim.run((0, 8), forces, moments, dt=0.02)

    # 播放动画
    print("\n观察要点：")
    print("1. 机头（尖端）应该始终指向飞行轨迹的切线方向")
    print("2. 飞机应该画出一个向右的弧线")
    print("3. 偏航角ψ应该持续增加（向右转）")

    sim.animate(speed=1.0)


def test_roll():
    """测试滚转 - 验证滚转方向"""
    print("=== 测试：滚转（验证滚转方向）===")

    mav = create_mav_from_params(ZagiParams)

    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 15.0
    initial_state[StateIndex.PD] = -50.0
    mav.set_state(initial_state)

    def forces(t, state):
        phi = state[StateIndex.PHI]
        theta = state[StateIndex.THETA]
        fg = mav.get_gravity_force_body(phi, theta)
        thrust = np.array([2.0, 0.0, 0.0])
        return fg + thrust

    # 滚转力矩 l > 0 -> 向右滚转（右翼下压）
    def moments(t, state):
        return np.array([0.5, 0.0, 0.0])

    sim = MAVSimulation(mav, viewer_scale=3.0)
    sim.run((0, 4), forces, moments, dt=0.02)

    print("\n观察要点：")
    print("1. 飞机应该向右滚转（右翼下压）")
    print("2. 滚转角φ应该持续增加")

    sim.animate(speed=1.0)


def test_pitch():
    """测试俯仰 - 验证俯仰方向"""
    print("=== 测试：俯仰（验证俯仰方向）===")

    mav = create_mav_from_params(ZagiParams)

    initial_state = np.zeros(12)
    initial_state[StateIndex.U] = 15.0
    initial_state[StateIndex.PD] = -100.0
    mav.set_state(initial_state)

    def forces(t, state):
        phi = state[StateIndex.PHI]
        theta = state[StateIndex.THETA]
        return mav.get_gravity_force_body(phi, theta)

    # 俯仰力矩 m > 0 -> 抬头
    def moments(t, state):
        return np.array([0.0, 0.3, 0.0])

    sim = MAVSimulation(mav, viewer_scale=3.0)
    sim.run((0, 4), forces, moments, dt=0.02)

    print("\n观察要点：")
    print("1. 飞机应该抬头（机头上仰）")
    print("2. 俯仰角θ应该持续增加")

    sim.animate(speed=1.0)


if __name__ == "__main__":
    print("=" * 50)
    print("机头朝向和姿态验证测试")
    print("=" * 50)
    print("\n选择测试：")
    print("1. 偏航转弯（验证机头朝向）")
    print("2. 滚转（验证滚转方向）")
    print("3. 俯仰（验证俯仰方向）")
    print("0. 退出")

    while True:
        try:
            choice = input("\n请选择 (0-3): ").strip()
            if choice == "0":
                break
            elif choice == "1":
                test_yaw_turn()
            elif choice == "2":
                test_roll()
            elif choice == "3":
                test_pitch()
            else:
                print("无效选择")
        except KeyboardInterrupt:
            break
