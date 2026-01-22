"""
MAV参数配置模块
包含Zagi飞翼和标准无人机的物理参数和气动参数
参考：《小型无人机理论与控制》附录E
"""

import numpy as np


class ZagiParams:
    """Zagi飞翼参数（表E.1）"""

    # 物理参数
    m = 1.56  # 质量 kg
    Jx = 0.1147  # x轴转动惯量 kg·m²
    Jy = 0.0576  # y轴转动惯量 kg·m²
    Jz = 0.1712  # z轴转动惯量 kg·m²
    Jxz = 0.0015  # 惯性积 kg·m²

    # 几何参数
    S = 0.2589  # 机翼面积 m²
    b = 1.4224  # 翼展 m
    c = 0.3302  # 平均气动弦长 m
    S_prop = 0.0314  # 螺旋桨面积 m²

    # 环境和推进参数
    rho = 1.2682  # 空气密度 kg/m³
    k_motor = 20  # 电机常数
    k_Tp = 0  # 推力常数
    k_Omega = 0  # 角速度常数
    e = 0.9  # Oswald效率因子

    # 纵向气动系数
    C_L0 = 0.09167
    C_D0 = 0.01631
    C_m0 = -0.02338
    C_L_alpha = 3.5016
    C_D_alpha = 0.2108
    C_m_alpha = -0.5675
    C_L_q = 2.8932
    C_D_q = 0
    C_m_q = -1.3990
    C_L_delta_e = 0.2724
    C_D_delta_e = 0.3045
    C_m_delta_e = -0.3254
    C_prop = 1.0
    M = 50
    alpha0 = 0.4712
    epsilon = 0.1592
    C_D_p = 0.0254

    # 横向气动系数
    C_Y0 = 0
    C_l0 = 0
    C_n0 = 0
    C_Y_beta = -0.07359
    C_l_beta = -0.02854
    C_n_beta = 0.00040
    C_Y_p = 0
    C_l_p = -0.3209
    C_n_p = -0.01297
    C_Y_r = 0
    C_l_r = 0.03066
    C_n_r = -0.00434
    C_Y_delta_a = 0
    C_l_delta_a = 0.1682
    C_n_delta_a = -0.00328


class UAVParams:
    """标准无人机参数（表E.2）"""

    # 物理参数
    m = 13.5  # 质量 kg
    Jx = 0.8244  # x轴转动惯量 kg·m²
    Jy = 1.135  # y轴转动惯量 kg·m²
    Jz = 1.759  # z轴转动惯量 kg·m²
    Jxz = 0.1204  # 惯性积 kg·m²

    # 几何参数
    S = 0.55  # 机翼面积 m²
    b = 2.8956  # 翼展 m
    c = 0.18994  # 平均气动弦长 m
    S_prop = 0.2027  # 螺旋桨面积 m²

    # 环境和推进参数
    rho = 1.2682  # 空气密度 kg/m³
    k_motor = 80  # 电机常数
    k_Tp = 0  # 推力常数
    k_Omega = 0  # 角速度常数
    e = 0.9  # Oswald效率因子

    # 纵向气动系数
    C_L0 = 0.28
    C_D0 = 0.03
    C_m0 = -0.02338
    C_L_alpha = 3.45
    C_D_alpha = 0.3
    C_m_alpha = -0.38
    C_L_q = 0
    C_D_q = 0
    C_m_q = -3.6
    C_L_delta_e = -0.36
    C_D_delta_e = 0
    C_m_delta_e = -0.5
    C_prop = 1.0
    M = 50
    alpha0 = 0.4712
    epsilon = 0.1592
    C_D_p = 0.0437
    C_n_delta_e = -0.032

    # 横向气动系数
    C_Y0 = 0
    C_l0 = 0
    C_n0 = 0
    C_Y_beta = -0.98
    C_l_beta = -0.12
    C_n_beta = 0.25
    C_Y_p = 0
    C_l_p = -0.26
    C_n_p = 0.022
    C_Y_r = 0
    C_l_r = 0.14
    C_n_r = -0.35
    C_Y_delta_a = 0
    C_l_delta_a = 0.08
    C_n_delta_a = 0.06
    C_Y_delta_r = -0.17
    C_l_delta_r = 0.105


def compute_gamma_coefficients(Jx, Jy, Jz, Jxz):
    """
    计算转动动力学方程中的Gamma系数

    参数:
        Jx, Jy, Jz: 转动惯量
        Jxz: 惯性积

    返回:
        dict: Gamma系数字典
    """
    Gamma = Jx * Jz - Jxz**2

    Gamma1 = Jxz * (Jx - Jy + Jz) / Gamma
    Gamma2 = (Jz * (Jz - Jy) + Jxz**2) / Gamma
    Gamma3 = Jz / Gamma
    Gamma4 = Jxz / Gamma
    Gamma5 = (Jz - Jx) / Jy
    Gamma6 = Jxz / Jy
    Gamma7 = ((Jx - Jy) * Jx + Jxz**2) / Gamma
    Gamma8 = Jx / Gamma

    return {
        "Gamma": Gamma,
        "Gamma1": Gamma1,
        "Gamma2": Gamma2,
        "Gamma3": Gamma3,
        "Gamma4": Gamma4,
        "Gamma5": Gamma5,
        "Gamma6": Gamma6,
        "Gamma7": Gamma7,
        "Gamma8": Gamma8,
    }


# 重力加速度
g = 9.81  # m/s²


if __name__ == "__main__":
    # 测试Gamma系数计算
    print("=== Zagi飞翼 Gamma系数 ===")
    gamma_zagi = compute_gamma_coefficients(
        ZagiParams.Jx, ZagiParams.Jy, ZagiParams.Jz, ZagiParams.Jxz
    )
    for k, v in gamma_zagi.items():
        print(f"  {k}: {v:.6f}")

    print("\n=== 标准无人机 Gamma系数 ===")
    gamma_uav = compute_gamma_coefficients(
        UAVParams.Jx, UAVParams.Jy, UAVParams.Jz, UAVParams.Jxz
    )
    for k, v in gamma_uav.items():
        print(f"  {k}: {v:.6f}")
