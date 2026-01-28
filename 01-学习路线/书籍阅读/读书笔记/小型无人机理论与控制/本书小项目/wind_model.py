"""
风速模型模块
实现稳定风和德莱登（Dryden）干扰风模型

参考：《小型无人机理论与控制》第四章
"""

import numpy as np
from scipy import signal


class DrydenWindModel:
    """
    德莱登风速模型

    将白噪声通过传递函数转换为具有真实统计特性的干扰风
    """

    # 德莱登风速模型参数（表4.1）
    TURBULENCE_PARAMS = {
        'light_low': {  # 高度低、轻度干扰
            'altitude': 50,
            'Lu': 200, 'Lv': 200, 'Lw': 50,
            'sigma_u': 1.06, 'sigma_v': 1.06, 'sigma_w': 0.7
        },
        'moderate_low': {  # 高度低、中度干扰
            'altitude': 50,
            'Lu': 200, 'Lv': 200, 'Lw': 50,
            'sigma_u': 2.12, 'sigma_v': 2.12, 'sigma_w': 1.4
        },
        'light_medium': {  # 高度中等、轻度干扰
            'altitude': 600,
            'Lu': 533, 'Lv': 533, 'Lw': 533,
            'sigma_u': 1.5, 'sigma_v': 1.5, 'sigma_w': 1.5
        },
        'moderate_medium': {  # 高度中等、中度干扰
            'altitude': 600,
            'Lu': 533, 'Lv': 533, 'Lw': 533,
            'sigma_u': 3.0, 'sigma_v': 3.0, 'sigma_w': 3.0
        }
    }

    def __init__(self, Va, turbulence_level='light_low', dt=0.01):
        """
        初始化德莱登风速模型

        参数:
            Va: 名义空速 (m/s)
            turbulence_level: 湍流等级 ('light_low', 'moderate_low', 'light_medium', 'moderate_medium')
            dt: 仿真时间步长 (s)
        """
        self.Va = Va
        self.dt = dt

        # 获取湍流参数
        params = self.TURBULENCE_PARAMS.get(turbulence_level, self.TURBULENCE_PARAMS['light_low'])
        self.Lu = params['Lu']
        self.Lv = params['Lv']
        self.Lw = params['Lw']
        self.sigma_u = params['sigma_u']
        self.sigma_v = params['sigma_v']
        self.sigma_w = params['sigma_w']

        # 创建离散传递函数
        self._create_discrete_filters()

        # 滤波器状态
        self.reset()

    def _create_discrete_filters(self):
        """创建离散化的德莱登传递函数"""
        Va = max(self.Va, 1.0)  # 避免除零

        # Hu(s) = σu * sqrt(2*Va/Lu) * 1/(s + Va/Lu)
        # 这是一阶低通滤波器
        num_u = [self.sigma_u * np.sqrt(2 * Va / self.Lu)]
        den_u = [1, Va / self.Lu]
        self.sys_u = signal.TransferFunction(num_u, den_u)
        self.sys_u_d = self.sys_u.to_discrete(self.dt, method='bilinear')

        # Hv(s) = σv * sqrt(3*Va/Lv) * (s + Va/(sqrt(3)*Lv)) / (s + Va/Lv)^2
        # 这是二阶系统
        a_v = Va / self.Lv
        b_v = Va / (np.sqrt(3) * self.Lv)
        gain_v = self.sigma_v * np.sqrt(3 * Va / self.Lv)
        num_v = [gain_v, gain_v * b_v]
        den_v = [1, 2 * a_v, a_v**2]
        self.sys_v = signal.TransferFunction(num_v, den_v)
        self.sys_v_d = self.sys_v.to_discrete(self.dt, method='bilinear')

        # Hw(s) = σw * sqrt(3*Va/Lw) * (s + Va/(sqrt(3)*Lw)) / (s + Va/Lw)^2
        a_w = Va / self.Lw
        b_w = Va / (np.sqrt(3) * self.Lw)
        gain_w = self.sigma_w * np.sqrt(3 * Va / self.Lw)
        num_w = [gain_w, gain_w * b_w]
        den_w = [1, 2 * a_w, a_w**2]
        self.sys_w = signal.TransferFunction(num_w, den_w)
        self.sys_w_d = self.sys_w.to_discrete(self.dt, method='bilinear')

    def reset(self):
        """重置滤波器状态"""
        # 初始化滤波器状态
        self.zi_u = np.zeros(max(len(self.sys_u_d.den) - 1, 1))
        self.zi_v = np.zeros(max(len(self.sys_v_d.den) - 1, 1))
        self.zi_w = np.zeros(max(len(self.sys_w_d.den) - 1, 1))

    def update(self):
        """
        更新风速（单步）

        返回:
            [u_wg, v_wg, w_wg]: 机体坐标系下的干扰风分量 (m/s)
        """
        # 生成白噪声输入
        noise_u = np.random.randn()
        noise_v = np.random.randn()
        noise_w = np.random.randn()

        # 通过离散滤波器
        u_wg, self.zi_u = signal.lfilter(
            self.sys_u_d.num, self.sys_u_d.den, [noise_u], zi=self.zi_u
        )
        v_wg, self.zi_v = signal.lfilter(
            self.sys_v_d.num, self.sys_v_d.den, [noise_v], zi=self.zi_v
        )
        w_wg, self.zi_w = signal.lfilter(
            self.sys_w_d.num, self.sys_w_d.den, [noise_w], zi=self.zi_w
        )

        return np.array([u_wg[0], v_wg[0], w_wg[0]])


class WindModel:
    """
    完整风速模型

    包括稳定风和干扰风（阵风）
    """

    def __init__(self, Va=20, steady_wind_ned=None, turbulence_level='light_low', dt=0.01):
        """
        初始化风速模型

        参数:
            Va: 名义空速 (m/s)
            steady_wind_ned: 稳定风在NED坐标系的分量 [wn, we, wd] (m/s)
            turbulence_level: 湍流等级，None表示无干扰风
            dt: 仿真时间步长 (s)
        """
        # 稳定风（NED坐标系）
        if steady_wind_ned is None:
            self.steady_wind_ned = np.zeros(3)
        else:
            self.steady_wind_ned = np.array(steady_wind_ned)

        # 干扰风模型
        self.turbulence_level = turbulence_level
        if turbulence_level is not None:
            self.dryden = DrydenWindModel(Va, turbulence_level, dt)
        else:
            self.dryden = None

        # 当前干扰风
        self.gust_body = np.zeros(3)

    def rotation_matrix_inertial_to_body(self, phi, theta, psi):
        """
        计算从惯性坐标系到机体坐标系的旋转矩阵 R_v^b

        参数:
            phi: 滚转角 (rad)
            theta: 俯仰角 (rad)
            psi: 偏航角 (rad)

        返回:
            3x3旋转矩阵
        """
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)

        # R_v^b = (R_b^v)^T
        R = np.array([
            [c_theta * c_psi, c_theta * s_psi, -s_theta],
            [s_phi * s_theta * c_psi - c_phi * s_psi,
             s_phi * s_theta * s_psi + c_phi * c_psi,
             s_phi * c_theta],
            [c_phi * s_theta * c_psi + s_phi * s_psi,
             c_phi * s_theta * s_psi - s_phi * c_psi,
             c_phi * c_theta]
        ])
        return R

    def update(self, phi, theta, psi):
        """
        更新并获取机体坐标系下的风速

        参数:
            phi, theta, psi: 欧拉角 (rad)

        返回:
            wind_body: 机体坐标系下的风速 [uw, vw, ww] (m/s)
            wind_ned: NED坐标系下的风速 [wn, we, wd] (m/s)
        """
        # 稳定风转换到机体坐标系
        R_v_b = self.rotation_matrix_inertial_to_body(phi, theta, psi)
        steady_wind_body = R_v_b @ self.steady_wind_ned

        # 更新干扰风
        if self.dryden is not None:
            self.gust_body = self.dryden.update()
        else:
            self.gust_body = np.zeros(3)

        # 总风速（机体坐标系）
        wind_body = steady_wind_body + self.gust_body

        # 总风速（NED坐标系）- 用于输出
        # 稳定风已经是NED，干扰风需要转换回去
        R_b_v = R_v_b.T
        gust_ned = R_b_v @ self.gust_body
        wind_ned = self.steady_wind_ned + gust_ned

        return wind_body, wind_ned

    def get_wind_body(self, phi, theta, psi):
        """
        获取机体坐标系下的风速（不更新干扰风）

        参数:
            phi, theta, psi: 欧拉角 (rad)

        返回:
            wind_body: [uw, vw, ww] (m/s)
        """
        R_v_b = self.rotation_matrix_inertial_to_body(phi, theta, psi)
        steady_wind_body = R_v_b @ self.steady_wind_ned
        return steady_wind_body + self.gust_body

    def set_steady_wind(self, wn, we, wd):
        """
        设置稳定风（NED坐标系）

        参数:
            wn: 北向风速 (m/s)
            we: 东向风速 (m/s)
            wd: 下向风速 (m/s)
        """
        self.steady_wind_ned = np.array([wn, we, wd])

    def reset(self):
        """重置风速模型"""
        if self.dryden is not None:
            self.dryden.reset()
        self.gust_body = np.zeros(3)


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    print("=== 德莱登风速模型测试 ===")

    # 创建风速模型
    Va = 20.0
    dt = 0.01
    duration = 30.0  # 仿真30秒

    wind_model = WindModel(
        Va=Va,
        steady_wind_ned=[3, 2, 0],  # 稳定风：北风3m/s，东风2m/s
        turbulence_level='moderate_low',
        dt=dt
    )

    # 仿真
    t = np.arange(0, duration, dt)
    wind_history = []
    gust_history = []

    # 假设飞机保持平飞
    phi, theta, psi = 0, 0, 0

    for _ in t:
        wind_body, wind_ned = wind_model.update(phi, theta, psi)
        wind_history.append(wind_body.copy())
        gust_history.append(wind_model.gust_body.copy())

    wind_history = np.array(wind_history)
    gust_history = np.array(gust_history)

    # 绘图
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('Dryden Wind Model Test', fontsize=14)

    labels = ['u_w (longitudinal)', 'v_w (lateral)', 'w_w (vertical)']
    for i in range(3):
        # 总风速
        ax = axes[i, 0]
        ax.plot(t, wind_history[:, i], 'b-', linewidth=0.5)
        ax.set_ylabel(f'{labels[i]} (m/s)')
        ax.set_title(f'Total Wind - {labels[i]}')
        ax.grid(True, alpha=0.3)
        if i == 2:
            ax.set_xlabel('Time (s)')

        # 干扰风
        ax = axes[i, 1]
        ax.plot(t, gust_history[:, i], 'r-', linewidth=0.5)
        ax.set_ylabel(f'{labels[i]} (m/s)')
        ax.set_title(f'Gust Only - {labels[i]}')
        ax.grid(True, alpha=0.3)
        if i == 2:
            ax.set_xlabel('Time (s)')

    plt.tight_layout()
    plt.show()

    # 统计信息
    print("\n=== 干扰风统计 ===")
    print(f"u_wg: mean={gust_history[:, 0].mean():.3f}, std={gust_history[:, 0].std():.3f}")
    print(f"v_wg: mean={gust_history[:, 1].mean():.3f}, std={gust_history[:, 1].std():.3f}")
    print(f"w_wg: mean={gust_history[:, 2].mean():.3f}, std={gust_history[:, 2].std():.3f}")
