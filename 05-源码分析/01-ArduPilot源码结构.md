# ArduPilot 源码结构总览

> 基于 ardupilot-master 目录分析

## 项目概述

ArduPilot 是一个成熟的开源飞控项目，代码结构清晰，模块化设计。

**源码位置**：`E:\A_MY_Projects\飞控参数学习包\ardupilot-master`

---

## 目录结构图

```
ardupilot-master/
│
├── ArduPlane/          # ★ 固定翼飞机代码（我们的学习重点）
├── ArduCopter/         # 多旋翼代码
├── ArduSub/            # 水下机器人代码
├── Rover/              # 地面车辆代码
├── AntennaTracker/     # 天线追踪器代码
├── Blimp/              # 飞艇代码
│
├── libraries/          # ★ 核心库（153个模块）
│
├── Tools/              # 工具脚本
│   ├── autotest/       # 自动测试/SITL仿真脚本
│   ├── environment_install/  # 环境安装脚本
│   └── ...
│
├── modules/            # 外部子模块
├── tests/              # 单元测试
├── benchmarks/         # 性能基准测试
│
├── Dockerfile          # Docker 构建文件
├── waf                 # 构建系统入口
├── wscript             # 构建配置
└── BUILD.md            # 构建说明
```

---

## ArduPlane 目录详解（重点）

`ArduPlane/` 是固定翼飞机的主程序目录。

### 核心文件

| 文件 | 作用 | 大小 |
|------|------|------|
| **Plane.cpp** | 主程序入口，初始化和主循环 | 34KB |
| **Plane.h** | 主类定义，包含所有状态变量 | 45KB |
| **Parameters.cpp** | 参数定义和默认值 | 85KB |
| **Parameters.h** | 参数声明 | 19KB |

### 飞行模式文件 (mode_*.cpp)

| 文件 | 飞行模式 | 说明 |
|------|----------|------|
| mode_manual.cpp | MANUAL | 手动模式，遥控器直接控制 |
| mode_stabilize.cpp | STABILIZE | 自稳模式，自动保持水平 |
| mode_fbwa.cpp | FBWA | 线控增稳A，限制倾斜角度 |
| mode_fbwb.cpp | FBWB | 线控增稳B，限制爬升率 |
| mode_auto.cpp | AUTO | 自动航线飞行 |
| mode_rtl.cpp | RTL | 返航模式 |
| mode_loiter.cpp | LOITER | 盘旋模式 |
| mode_guided.cpp | GUIDED | 引导模式 |
| mode_cruise.cpp | CRUISE | 巡航模式 |
| mode_acro.cpp | ACRO | 特技模式 |
| mode_takeoff.cpp | TAKEOFF | 自动起飞 |
| mode_thermal.cpp | THERMAL | 热气流滑翔 |

### 控制相关文件

| 文件 | 功能 |
|------|------|
| **Attitude.cpp** | 姿态控制算法（29KB） |
| **altitude.cpp** | 高度控制（32KB） |
| **navigation.cpp** | 导航控制（19KB） |
| **servos.cpp** | 舵机输出控制（46KB） |
| **radio.cpp** | 遥控器输入处理（14KB） |

### 安全相关文件

| 文件 | 功能 |
|------|------|
| failsafe.cpp | 失控保护 |
| fence.cpp | 地理围栏 |
| ekf_check.cpp | 导航滤波器检查 |
| AP_Arming_Plane.cpp | 解锁检查 |

### 特殊功能文件

| 文件 | 功能 |
|------|------|
| **quadplane.cpp** | 垂直起降四旋翼（192KB，最大文件） |
| tailsitter.cpp | 尾座式飞机 |
| tiltrotor.cpp | 倾转旋翼 |
| soaring.cpp | 滑翔/热气流利用 |

---

## libraries 目录（核心库）

共 153 个库模块，以下是最重要的：

### 姿态控制相关

| 库 | 功能 |
|---|------|
| **AC_AttitudeControl** | 姿态控制器 |
| **AC_PID** | PID 控制器实现 |
| **AC_AutoTune** | 自动调参 |

### 传感器相关

| 库 | 功能 |
|---|------|
| **AP_AHRS** | 姿态航向参考系统（核心） |
| **AP_InertialSensor** | IMU（加速度计+陀螺仪） |
| **AP_Compass** | 罗盘/磁力计 |
| **AP_Baro** | 气压计 |
| **AP_GPS** | GPS 模块 |
| **AP_Airspeed** | 空速计 |

### 导航相关

| 库 | 功能 |
|---|------|
| **AP_NavEKF3** | 扩展卡尔曼滤波器（导航核心） |
| **AP_Mission** | 航线任务管理 |
| **AC_WPNav** | 航点导航 |

### 输出控制

| 库 | 功能 |
|---|------|
| **SRV_Channel** | 舵机通道 |
| **AP_Motors** | 电机控制 |

### 通信相关

| 库 | 功能 |
|---|------|
| **GCS_MAVLink** | MAVLink 通信协议 |
| **AP_HAL** | 硬件抽象层 |

### 其他重要库

| 库 | 功能 |
|---|------|
| **AP_Param** | 参数系统 |
| **AP_Logger** | 日志记录 |
| **AP_Scheduler** | 任务调度 |
| **AP_TECS** | 总能量控制（固定翼专用） |
| **AP_L1_Control** | L1 导航控制器（固定翼专用） |

---

## Tools 目录

### 重要工具

| 目录 | 用途 |
|------|------|
| **autotest/** | SITL 仿真脚本 |
| **environment_install/** | 环境安装脚本 |
| **Frame_params/** | 各种机架的默认参数 |
| **Replay/** | 日志回放工具 |
| **scripts/** | 各种辅助脚本 |

### SITL 仿真入口

```
Tools/autotest/
├── arduplane.py      # ArduPlane 仿真脚本
├── sim_vehicle.py    # 通用仿真启动脚本
└── autotest.py       # 自动测试主脚本
```

---

## 构建系统

ArduPilot 使用 **Waf** 构建系统。

### 常用命令

```bash
# 配置 SITL 仿真构建
./waf configure --board sitl

# 构建 ArduPlane
./waf plane

# 使用 Docker 构建
docker build --rm -t ardupilot-dev .
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf plane
```

---

## 学习建议顺序

作为初学者，建议按以下顺序逐步深入：

1. **Parameters.cpp** - 了解有哪些参数
2. **mode_*.cpp** - 理解各飞行模式
3. **Attitude.cpp** - 姿态控制原理
4. **AP_AHRS** - 姿态估计
5. **AP_TECS** - 固定翼能量控制

---

## 代码规模统计

| 目录 | 文件数 | 说明 |
|------|--------|------|
| ArduPlane | 88 | 固定翼主程序 |
| libraries | 153 | 核心库模块 |
| 总计 | 240+ | C++ 源文件 |

---

*文档创建时间：2026-01-15*
*基于 ardupilot-master 分析*
