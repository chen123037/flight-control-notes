"""
气动系数计算模块
实现升力系数CL(α)、阻力系数CD(α)以及相关的坐标变换

参考：《小型无人机理论与控制》第四章 式(4.9)-(4.19)
"""

import numpy as np


def sigmoid_blend(alpha, M, alpha0):
    """
    计算Sigmoid混合函数σ(α) - 式(4.10)

    用于在线性升力模型和失速后平板模型之间平滑过渡

    参数:
        alpha: 攻角 (rad)
        M: 转换速率（正常数，值越大过渡越陡峭）
        alpha0: 失速攻角（分界点）

    返回:
        sigma: 混合系数，0表示线性模型，1表示平板模型
    """
    # 避免数值溢出
    exp_neg = np.clip(-M * (alpha - alpha0), -500, 500)
    exp_pos = np.clip(M * (alpha + alpha0), -500, 500)

    numerator = 1 + np.exp(exp_neg) + np.exp(exp_pos)
    denominator = (1 + np.exp(exp_neg)) * (1 + np.exp(exp_pos))

    # 避免除零
    if denominator == 0:
        return 0.0

    return numerator / denominator


def compute_CL(alpha, C_L0, C_L_alpha, M, alpha0):
    """
    计算升力系数CL(α) - 式(4.9)

    使用混合模型：小攻角时为线性模型，大攻角时为平板模型

    参数:
        alpha: 攻角 (rad)
        C_L0: 零攻角升力系数
        C_L_alpha: 升力线斜率
        M: sigmoid转换速率
        alpha0: 失速攻角

    返回:
        C_L: 升力系数
    """
    sigma = sigmoid_blend(alpha, M, alpha0)

    # 线性部分
    C_L_linear = C_L0 + C_L_alpha * alpha

    # 平板模型（失速后）
    # 2 * sign(α) * sin²(α) * cos(α)
    C_L_flat_plate = 2 * np.sign(alpha) * (np.sin(alpha) ** 2) * np.cos(alpha)

    # 混合
    C_L = (1 - sigma) * C_L_linear + sigma * C_L_flat_plate

    return C_L


def compute_CD(alpha, C_D_p, C_L0, C_L_alpha, e, AR):
    """
    计算阻力系数CD(α) - 式(4.11)

    阻力 = 废阻力 + 诱导阻力

    参数:
        alpha: 攻角 (rad)
        C_D_p: 废阻力系数（寄生阻力）
        C_L0: 零攻角升力系数
        C_L_alpha: 升力线斜率
        e: 奥斯瓦尔德效率因数 (0.8~1.0)
        AR: 展弦比 (b²/S)

    返回:
        C_D: 阻力系数
    """
    # 升力（用于诱导阻力计算）
    C_L_linear = C_L0 + C_L_alpha * alpha

    # 诱导阻力
    C_D_induced = (C_L_linear ** 2) / (np.pi * e * AR)

    # 总阻力
    C_D = C_D_p + C_D_induced

    return C_D


def compute_CL_linear(alpha, C_L0, C_L_alpha):
    """
    简化线性升力模型 - 式(4.12)

    参数:
        alpha: 攻角 (rad)
        C_L0: 零攻角升力系数
        C_L_alpha: 升力线斜率

    返回:
        C_L: 升力系数
    """
    return C_L0 + C_L_alpha * alpha


def compute_CD_linear(alpha, C_D0, C_D_alpha):
    """
    简化线性阻力模型 - 式(4.13)

    参数:
        alpha: 攻角 (rad)
        C_D0: 零攻角阻力系数
        C_D_alpha: 阻力系数对攻角的导数

    返回:
        C_D: 阻力系数
    """
    return C_D0 + C_D_alpha * alpha


def stability_to_body(C_L, C_D, alpha):
    """
    将稳定坐标系的升力/阻力系数转换为机体坐标系 - 式(4.19)

    稳定坐标系：升力垂直于来流，阻力平行于来流
    机体坐标系：X轴指向机头，Z轴指向机腹

    参数:
        C_L: 升力系数
        C_D: 阻力系数
        alpha: 攻角 (rad)

    返回:
        C_X: 机体X轴方向的力系数
        C_Z: 机体Z轴方向的力系数
    """
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    # 式(4.19)
    C_X = -C_D * cos_alpha + C_L * sin_alpha
    C_Z = -C_D * sin_alpha - C_L * cos_alpha

    return C_X, C_Z


def compute_C_X_coefficients(alpha, C_L0, C_L_alpha, C_D_p, e, AR, M, alpha0,
                              C_L_q, C_D_q, C_L_delta_e, C_D_delta_e):
    """
    计算机体X轴方向的所有气动系数

    参数:
        alpha: 攻角 (rad)
        其他参数：气动导数

    返回:
        C_X: 基础系数
        C_X_q: 俯仰角速度相关系数
        C_X_delta_e: 升降舵相关系数
    """
    # 计算基础升力和阻力
    C_L = compute_CL(alpha, C_L0, C_L_alpha, M, alpha0)
    C_D = compute_CD(alpha, C_D_p, C_L0, C_L_alpha, e, AR)

    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    # C_X(α)
    C_X = -C_D * cos_alpha + C_L * sin_alpha

    # C_X_q(α)
    C_X_q = -C_D_q * cos_alpha + C_L_q * sin_alpha

    # C_X_δe(α)
    C_X_delta_e = -C_D_delta_e * cos_alpha + C_L_delta_e * sin_alpha

    return C_X, C_X_q, C_X_delta_e


def compute_C_Z_coefficients(alpha, C_L0, C_L_alpha, C_D_p, e, AR, M, alpha0,
                              C_L_q, C_D_q, C_L_delta_e, C_D_delta_e):
    """
    计算机体Z轴方向的所有气动系数

    参数:
        alpha: 攻角 (rad)
        其他参数：气动导数

    返回:
        C_Z: 基础系数
        C_Z_q: 俯仰角速度相关系数
        C_Z_delta_e: 升降舵相关系数
    """
    # 计算基础升力和阻力
    C_L = compute_CL(alpha, C_L0, C_L_alpha, M, alpha0)
    C_D = compute_CD(alpha, C_D_p, C_L0, C_L_alpha, e, AR)

    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    # C_Z(α)
    C_Z = -C_D * sin_alpha - C_L * cos_alpha

    # C_Z_q(α)
    C_Z_q = -C_D_q * sin_alpha - C_L_q * cos_alpha

    # C_Z_δe(α)
    C_Z_delta_e = -C_D_delta_e * sin_alpha - C_L_delta_e * cos_alpha

    return C_Z, C_Z_q, C_Z_delta_e


def compute_aspect_ratio(b, S):
    """
    计算展弦比

    参数:
        b: 翼展 (m)
        S: 机翼面积 (m²)

    返回:
        AR: 展弦比
    """
    return b ** 2 / S


def compute_CL_alpha_theoretical(AR):
    """
    根据展弦比计算理论升力线斜率

    参数:
        AR: 展弦比

    返回:
        C_L_alpha: 升力线斜率 (1/rad)
    """
    return np.pi * AR / (1 + np.sqrt(1 + (AR / 2) ** 2))


class AerodynamicsModel:
    """气动模型类，封装所有气动计算"""

    def __init__(self, params):
        """
        初始化气动模型

        参数:
            params: 参数类（ZagiParams或UAVParams）
        """
        self.params = params

        # 计算展弦比
        self.AR = compute_aspect_ratio(params.b, params.S)

        # 缓存常用参数
        self.C_L0 = params.C_L0
        self.C_L_alpha = params.C_L_alpha
        self.C_D_p = params.C_D_p
        self.e = params.e
        self.M = params.M
        self.alpha0 = params.alpha0

        # 升力/阻力导数
        self.C_L_q = params.C_L_q
        self.C_D_q = params.C_D_q
        self.C_L_delta_e = params.C_L_delta_e
        self.C_D_delta_e = params.C_D_delta_e

    def CL(self, alpha):
        """计算升力系数"""
        return compute_CL(alpha, self.C_L0, self.C_L_alpha,
                         self.M, self.alpha0)

    def CD(self, alpha):
        """计算阻力系数"""
        return compute_CD(alpha, self.C_D_p, self.C_L0, self.C_L_alpha,
                         self.e, self.AR)

    def get_longitudinal_coefficients(self, alpha):
        """
        获取纵向气动系数（机体坐标系）

        返回:
            dict: 包含C_X, C_X_q, C_X_delta_e, C_Z, C_Z_q, C_Z_delta_e
        """
        C_X, C_X_q, C_X_delta_e = compute_C_X_coefficients(
            alpha, self.C_L0, self.C_L_alpha, self.C_D_p, self.e, self.AR,
            self.M, self.alpha0, self.C_L_q, self.C_D_q,
            self.C_L_delta_e, self.C_D_delta_e
        )

        C_Z, C_Z_q, C_Z_delta_e = compute_C_Z_coefficients(
            alpha, self.C_L0, self.C_L_alpha, self.C_D_p, self.e, self.AR,
            self.M, self.alpha0, self.C_L_q, self.C_D_q,
            self.C_L_delta_e, self.C_D_delta_e
        )

        return {
            'C_X': C_X, 'C_X_q': C_X_q, 'C_X_delta_e': C_X_delta_e,
            'C_Z': C_Z, 'C_Z_q': C_Z_q, 'C_Z_delta_e': C_Z_delta_e
        }


if __name__ == "__main__":
    # 测试气动系数计算
    from mav_params import ZagiParams

    print("=== 气动系数测试 ===")

    # 创建气动模型
    aero = AerodynamicsModel(ZagiParams)

    print(f"\n展弦比 AR = {aero.AR:.4f}")
    print(f"理论升力线斜率 C_L_alpha = {compute_CL_alpha_theoretical(aero.AR):.4f}")
    print(f"实际升力线斜率 C_L_alpha = {ZagiParams.C_L_alpha:.4f}")

    # 测试不同攻角
    print("\n攻角(deg) | CL      | CD      | C_X     | C_Z")
    print("-" * 50)
    for alpha_deg in [-10, -5, 0, 5, 10, 15, 20, 30, 45]:
        alpha = np.deg2rad(alpha_deg)
        C_L = aero.CL(alpha)
        C_D = aero.CD(alpha)
        C_X, C_Z = stability_to_body(C_L, C_D, alpha)
        print(f"{alpha_deg:8.1f}  | {C_L:7.4f} | {C_D:7.4f} | {C_X:7.4f} | {C_Z:7.4f}")

    # 测试sigmoid函数
    print("\n=== Sigmoid混合函数测试 ===")
    print("攻角(deg) | σ(α)")
    print("-" * 25)
    for alpha_deg in [-30, -20, -10, 0, 10, 20, 30]:
        alpha = np.deg2rad(alpha_deg)
        sigma = sigmoid_blend(alpha, ZagiParams.M, ZagiParams.alpha0)
        print(f"{alpha_deg:8.1f}  | {sigma:.4f}")
