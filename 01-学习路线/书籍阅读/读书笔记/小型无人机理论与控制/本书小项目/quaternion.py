"""
四元数工具模块
提供四元数运算、转换和归一化功能

参考：《小型无人机理论与控制》附录B
"""

import numpy as np


class Quaternion:
    """四元数类"""

    def __init__(self, e0=1.0, e1=0.0, e2=0.0, e3=0.0):
        """
        初始化四元数

        参数:
            e0: 标量部分
            e1, e2, e3: 矢量部分
        """
        self.e = np.array([e0, e1, e2, e3], dtype=float)

    @property
    def e0(self):
        return self.e[0]

    @property
    def e1(self):
        return self.e[1]

    @property
    def e2(self):
        return self.e[2]

    @property
    def e3(self):
        return self.e[3]

    @property
    def scalar(self):
        """标量部分"""
        return self.e[0]

    @property
    def vector(self):
        """矢量部分"""
        return self.e[1:4]

    def norm(self):
        """计算四元数范数"""
        return np.sqrt(np.sum(self.e ** 2))

    def normalize(self):
        """归一化四元数"""
        n = self.norm()
        if n > 1e-10:
            self.e /= n
        return self

    def normalized(self):
        """返回归一化后的新四元数"""
        n = self.norm()
        if n > 1e-10:
            return Quaternion(*self.e / n)
        return Quaternion(*self.e)

    def conjugate(self):
        """共轭四元数"""
        return Quaternion(self.e0, -self.e1, -self.e2, -self.e3)

    def inverse(self):
        """逆四元数"""
        n2 = np.sum(self.e ** 2)
        if n2 > 1e-10:
            conj = self.conjugate()
            return Quaternion(*conj.e / n2)
        return self.conjugate()

    def __mul__(self, other):
        """四元数乘法"""
        if isinstance(other, Quaternion):
            # 四元数乘法
            e0 = self.e0 * other.e0 - self.e1 * other.e1 - self.e2 * other.e2 - self.e3 * other.e3
            e1 = self.e0 * other.e1 + self.e1 * other.e0 + self.e2 * other.e3 - self.e3 * other.e2
            e2 = self.e0 * other.e2 - self.e1 * other.e3 + self.e2 * other.e0 + self.e3 * other.e1
            e3 = self.e0 * other.e3 + self.e1 * other.e2 - self.e2 * other.e1 + self.e3 * other.e0
            return Quaternion(e0, e1, e2, e3)
        else:
            # 标量乘法
            return Quaternion(*self.e * other)

    def __repr__(self):
        return f"Quaternion({self.e0:.6f}, {self.e1:.6f}, {self.e2:.6f}, {self.e3:.6f})"


# ==================== 转换函数 ====================

def euler_to_quaternion(phi, theta, psi):
    """
    欧拉角转四元数

    参数:
        phi: 滚转角 (rad)
        theta: 俯仰角 (rad)
        psi: 偏航角 (rad)

    返回:
        numpy数组 [e0, e1, e2, e3]
    """
    # 半角
    c_phi = np.cos(phi / 2)
    s_phi = np.sin(phi / 2)
    c_theta = np.cos(theta / 2)
    s_theta = np.sin(theta / 2)
    c_psi = np.cos(psi / 2)
    s_psi = np.sin(psi / 2)

    # 四元数分量
    e0 = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi
    e1 = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi
    e2 = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi
    e3 = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi

    return np.array([e0, e1, e2, e3])


def quaternion_to_euler(e):
    """
    四元数转欧拉角

    参数:
        e: 四元数 [e0, e1, e2, e3] 或 Quaternion对象

    返回:
        (phi, theta, psi) 欧拉角 (rad)
    """
    if isinstance(e, Quaternion):
        e0, e1, e2, e3 = e.e
    else:
        e0, e1, e2, e3 = e

    # 滚转角 phi
    phi = np.arctan2(2 * (e0 * e1 + e2 * e3),
                     e0 ** 2 + e3 ** 2 - e1 ** 2 - e2 ** 2)

    # 俯仰角 theta (限制在[-pi/2, pi/2])
    sin_theta = 2 * (e0 * e2 - e1 * e3)
    sin_theta = np.clip(sin_theta, -1.0, 1.0)  # 防止数值误差
    theta = np.arcsin(sin_theta)

    # 偏航角 psi
    psi = np.arctan2(2 * (e0 * e3 + e1 * e2),
                     e0 ** 2 + e1 ** 2 - e2 ** 2 - e3 ** 2)

    return phi, theta, psi


# ==================== 旋转矩阵 ====================

def quaternion_to_rotation_matrix(e):
    """
    四元数转旋转矩阵 R_b^i（机体系到惯性系）

    参数:
        e: 四元数 [e0, e1, e2, e3]

    返回:
        3x3 旋转矩阵
    """
    if isinstance(e, Quaternion):
        e0, e1, e2, e3 = e.e
    else:
        e0, e1, e2, e3 = e

    # 式(B.1)
    R = np.array([
        [e1**2 + e0**2 - e2**2 - e3**2,  2*(e1*e2 - e3*e0),           2*(e1*e3 + e2*e0)],
        [2*(e1*e2 + e3*e0),              e2**2 + e0**2 - e1**2 - e3**2, 2*(e2*e3 - e1*e0)],
        [2*(e1*e3 - e2*e0),              2*(e2*e3 + e1*e0),           e3**2 + e0**2 - e1**2 - e2**2]
    ])

    return R


def rotation_matrix_body_to_inertial(e):
    """
    机体系到惯性系的旋转矩阵（同 quaternion_to_rotation_matrix）
    """
    return quaternion_to_rotation_matrix(e)


def rotation_matrix_inertial_to_body(e):
    """
    惯性系到机体系的旋转矩阵（旋转矩阵的转置）
    """
    return quaternion_to_rotation_matrix(e).T


# ==================== 四元数微分 ====================

def quaternion_derivative(e, p, q, r, lambda_norm=0.0):
    """
    计算四元数的时间导数 - 式(B.3)

    参数:
        e: 四元数 [e0, e1, e2, e3]
        p, q, r: 角速度 (rad/s)
        lambda_norm: 正交控制增益（0表示不使用，建议1000）

    返回:
        四元数导数 [e0_dot, e1_dot, e2_dot, e3_dot]
    """
    if isinstance(e, Quaternion):
        e0, e1, e2, e3 = e.e
    else:
        e0, e1, e2, e3 = e

    if lambda_norm > 0:
        # 科贝特-赖特正交控制
        norm_sq = e0**2 + e1**2 + e2**2 + e3**2
        lam = lambda_norm * (1 - norm_sq)

        # 修正的四元数微分矩阵
        Omega = 0.5 * np.array([
            [lam, -p, -q, -r],
            [p, lam, r, -q],
            [q, -r, lam, p],
            [r, q, -p, lam]
        ])
    else:
        # 标准四元数微分矩阵 - 式(B.3)
        Omega = 0.5 * np.array([
            [0, -p, -q, -r],
            [p, 0, r, -q],
            [q, -r, 0, p],
            [r, q, -p, 0]
        ])

    e_vec = np.array([e0, e1, e2, e3])
    return Omega @ e_vec


def quaternion_derivative_expanded(e, p, q, r):
    """
    四元数导数展开形式 - 式(B.11)~(B.14)

    参数:
        e: 四元数 [e0, e1, e2, e3]
        p, q, r: 角速度 (rad/s)

    返回:
        [e0_dot, e1_dot, e2_dot, e3_dot]
    """
    if isinstance(e, Quaternion):
        e0, e1, e2, e3 = e.e
    else:
        e0, e1, e2, e3 = e

    e0_dot = -0.5 * (p * e1 + q * e2 + r * e3)  # B.11
    e1_dot = 0.5 * (p * e0 + r * e2 - q * e3)   # B.12
    e2_dot = 0.5 * (q * e0 - r * e1 + p * e3)   # B.13
    e3_dot = 0.5 * (r * e0 + q * e1 - p * e2)   # B.14

    return np.array([e0_dot, e1_dot, e2_dot, e3_dot])


# ==================== 重力计算 ====================

def gravity_body_frame(e, m, g=9.81):
    """
    计算机体坐标系下的重力分量

    参数:
        e: 四元数 [e0, e1, e2, e3]
        m: 质量 (kg)
        g: 重力加速度 (m/s^2)

    返回:
        [fx_g, fy_g, fz_g] 重力在机体系的分量
    """
    if isinstance(e, Quaternion):
        e0, e1, e2, e3 = e.e
    else:
        e0, e1, e2, e3 = e

    fx_g = m * g * 2 * (e1 * e3 - e2 * e0)
    fy_g = m * g * 2 * (e2 * e3 + e1 * e0)
    fz_g = m * g * (e3**2 + e0**2 - e1**2 - e2**2)

    return np.array([fx_g, fy_g, fz_g])


# ==================== 归一化 ====================

def normalize_quaternion(e):
    """
    归一化四元数

    参数:
        e: 四元数 [e0, e1, e2, e3]

    返回:
        归一化后的四元数
    """
    if isinstance(e, Quaternion):
        return e.normalized().e
    else:
        e = np.array(e)
        norm = np.sqrt(np.sum(e ** 2))
        if norm > 1e-10:
            return e / norm
        return e


def quaternion_norm_error(e):
    """
    计算四元数范数误差

    参数:
        e: 四元数 [e0, e1, e2, e3]

    返回:
        |1 - ||e||| 范数误差
    """
    if isinstance(e, Quaternion):
        norm = e.norm()
    else:
        norm = np.sqrt(np.sum(np.array(e) ** 2))
    return abs(1.0 - norm)


# ==================== 向量旋转 ====================

def rotate_vector_by_quaternion(v, e):
    """
    使用四元数旋转向量（从机体系到惯性系）

    参数:
        v: 向量 [vx, vy, vz]（机体系）
        e: 四元数 [e0, e1, e2, e3]

    返回:
        旋转后的向量（惯性系）
    """
    R = quaternion_to_rotation_matrix(e)
    return R @ v


def rotate_vector_to_body(v, e):
    """
    使用四元数旋转向量（从惯性系到机体系）

    参数:
        v: 向量 [vx, vy, vz]（惯性系）
        e: 四元数 [e0, e1, e2, e3]

    返回:
        旋转后的向量（机体系）
    """
    R = quaternion_to_rotation_matrix(e)
    return R.T @ v


# ==================== 测试 ====================

if __name__ == "__main__":
    print("=" * 50)
    print("四元数模块测试")
    print("=" * 50)

    # 测试1: 欧拉角-四元数转换
    print("\n--- 测试1: 欧拉角-四元数转换 ---")
    test_angles = [
        (0, 0, 0),
        (np.deg2rad(30), 0, 0),
        (0, np.deg2rad(45), 0),
        (0, 0, np.deg2rad(60)),
        (np.deg2rad(30), np.deg2rad(45), np.deg2rad(60)),
    ]

    for phi, theta, psi in test_angles:
        e = euler_to_quaternion(phi, theta, psi)
        phi2, theta2, psi2 = quaternion_to_euler(e)

        print(f"原始: φ={np.rad2deg(phi):6.1f}°, θ={np.rad2deg(theta):6.1f}°, ψ={np.rad2deg(psi):6.1f}°")
        print(f"转换: φ={np.rad2deg(phi2):6.1f}°, θ={np.rad2deg(theta2):6.1f}°, ψ={np.rad2deg(psi2):6.1f}°")
        print(f"四元数: [{e[0]:.4f}, {e[1]:.4f}, {e[2]:.4f}, {e[3]:.4f}], 范数={np.linalg.norm(e):.6f}")
        print()

    # 测试2: 四元数范数
    print("\n--- 测试2: 四元数归一化 ---")
    e_unnorm = np.array([1.1, 0.1, 0.2, 0.3])
    print(f"未归一化: {e_unnorm}, 范数={np.linalg.norm(e_unnorm):.4f}")
    e_norm = normalize_quaternion(e_unnorm)
    print(f"归一化后: {e_norm}, 范数={np.linalg.norm(e_norm):.6f}")

    # 测试3: 重力计算对比
    print("\n--- 测试3: 重力计算（四元数 vs 欧拉角）---")
    m = 1.0
    g = 9.81

    for phi, theta, psi in [(0, 0, 0), (np.deg2rad(30), np.deg2rad(45), 0)]:
        # 四元数方法
        e = euler_to_quaternion(phi, theta, psi)
        fg_quat = gravity_body_frame(e, m, g)

        # 欧拉角方法（传统公式）
        fx_euler = -m * g * np.sin(theta)
        fy_euler = m * g * np.cos(theta) * np.sin(phi)
        fz_euler = m * g * np.cos(theta) * np.cos(phi)
        fg_euler = np.array([fx_euler, fy_euler, fz_euler])

        print(f"姿态: φ={np.rad2deg(phi):.0f}°, θ={np.rad2deg(theta):.0f}°")
        print(f"  四元数: [{fg_quat[0]:.4f}, {fg_quat[1]:.4f}, {fg_quat[2]:.4f}]")
        print(f"  欧拉角: [{fg_euler[0]:.4f}, {fg_euler[1]:.4f}, {fg_euler[2]:.4f}]")
        print(f"  误差:   {np.linalg.norm(fg_quat - fg_euler):.2e}")
        print()

    # 测试4: 四元数微分
    print("\n--- 测试4: 四元数微分 ---")
    e = euler_to_quaternion(0, 0, 0)
    p, q, r = np.deg2rad(10), np.deg2rad(5), np.deg2rad(3)

    e_dot_matrix = quaternion_derivative(e, p, q, r)
    e_dot_expand = quaternion_derivative_expanded(e, p, q, r)

    print(f"角速度: p={np.rad2deg(p):.1f}°/s, q={np.rad2deg(q):.1f}°/s, r={np.rad2deg(r):.1f}°/s")
    print(f"矩阵形式: {e_dot_matrix}")
    print(f"展开形式: {e_dot_expand}")
    print(f"差异: {np.linalg.norm(e_dot_matrix - e_dot_expand):.2e}")

    print("\n测试完成!")
