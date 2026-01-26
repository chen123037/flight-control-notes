# 第 1 周学习记录 - Mission Planner 深度使用

> 阶段 1 · 第 1 个月 · 第 1 周
> 日期：2026-01-19（周一）至 2026-01-23（周五）
> 周末休息

---

## 本周目标

**核心产出**：《Mission Planner 功能手册》

| 检查项 | 完成 |
|--------|------|
| [ ] 手册内容 ≥ 8 页 | |
| [ ] 包含各界面截图 | |
| [ ] 包含你自己的理解和注释 | |
| [ ] SITL 完成一次完整航线任务 | |
| [ ] 任务截图/录屏保存 | |

---

## 每日计划与记录

### 周一 1/19 - SITL 复习 + Flight Data

**任务**：
- 复习 SITL 启动命令
- 复习 Mission Planner 连接步骤
- 记录完整的启动流程

**产出**：启动流程文档（可作为手册第一章）

**记录**：
```
学习时长：4 小时
完成情况：
-- 完成 SITL 启动流程整理，已写入《Mission Planner 功能手册》第1章
- 保存 SITL 启动与 Mission Planner 连接成功截图
- 完成 Flight Data 页面学习，已补充手册第2章
遇到问题：
- 初次连接失败（忘记启动 SITL），启动后自动连接成功
- 路径旧配置导致 sim_vehicle.py 调用错误（已修复 PATH）
```

---

### 周二 1/20 - Flight Plan 界面

**任务**：
- 学习航点规划操作
- 了解各种航点命令类型
- 实际规划一条航线

**重点内容**：
- 航点添加/删除/编辑
- 常用命令：WAYPOINT, LOITER, RTL 等
- 航线保存/加载
- 地理围栏设置

**产出**：手册《航线规划指南》章节

**记录**：
```
学习时长：未记录
完成情况：
- 完成 Flight Plan 航线规划练习（添加航点、TAKEOFF、LOITER_TURNS、RTL）
- 完成写入/读取/保存航线，文件已保存到 `docs/06-实践记录/SITL/`
- 设置并保存 FENCE 围栏，保存截图
遇到问题：
- 读取航点提示将清空本地列表（属正常提示）
- 围栏绘制与类型选择初期不熟悉（已完成设置）
```

---

### 周三 1/21 - Initial Setup 界面

**任务**：
- 了解各种校准流程
- 了解机架配置选项
- 记录校准步骤

**重点内容**：
- 固件安装
- 机架类型选择
- 加速度计校准
- 罗盘校准
- 遥控器校准
- 飞行模式设置

**产出**：手册《初始设置与校准》章节

**记录**：
```
学习时长：早上（未记录具体时长）
完成情况：完成 Initial Setup 与 Flight Data 学习，手册已补齐对应章节
遇到问题：SITL 下校准易失败、罗盘进度 0、遥控通道不动、告警需重启（均为仿真正常现象）
```

---

### 周四 1/22 - Config/Tuning 界面

**任务**：
- 了解参数配置界面
- 浏览 ArduPlane 参数分类
- 找到 PID 相关参数位置

**重点内容**：
- Full Parameter List
- Basic Tuning
- Extended Tuning
- Planner 设置

**产出**：手册《参数配置界面》章节

**记录**：
```
学习时长：__ 小时
完成情况：补充 Config/Tuning 各页面说明（Basic/Extended/Onboard OSD/MAVFtp/User Params/Planner/CubeLan/全部参数表学习框架）
遇到问题：左侧菜单识别与含义需截图确认；部分扩展调参项全为 0 且不可调
```
提醒：参数学习清单（第5.2.1）未完成，明天/后天补学并记录

---

### 周五 1/23 - SITL 任务实战 + 整理

**任务**：
- 在 SITL 中规划并执行完整航线
- 整理本周内容，完成手册

**实战任务要求**：
1. 起飞（TAKEOFF）
2. 至少 3 个航点
3. 一次盘旋（LOITER）
4. 返航（RTL）

**产出**：
- 航线截图/录屏
- 完整的《Mission Planner 功能手册》

**记录**：
```
学习时长：未记录
完成情况：
- 完成 SITL 任务实战：TAKEOFF、≥3 航点、LOITER（2 圈）、返航与降落
- 保存航线截图/记录至 `docs/03-Mission-Planner/20260123飞行记录/`
- 航线文件保存至 `docs/06-实践记录/SITL/20260120 1st fly.waypoints`
遇到问题：
- RTL 返航后默认盘旋不降落（固定翼），需改为 LAND 流程
```

---

### 周六 1/24 & 周日 1/25 - 休息

休息日，可选择性复习或阅读书籍

---

## 本周产出物

### 1. Mission Planner 功能手册

存放位置：`docs/03-Mission-Planner/功能手册.md`

**目录结构**：
```
# Mission Planner 功能手册

## 第1章 环境准备与启动
## 第2章 Flight Data 界面详解
## 第3章 航线规划指南
## 第4章 初始设置与校准
## 第5章 参数配置界面
## 第6章 实战：完整航线任务
## 附录：常用快捷键/常见问题
```

### 2. SITL 任务记录

存放位置：`docs/06-实践记录/SITL/`

---

## 本周学习资源

- ArduPilot Plane 文档：https://ardupilot.org/plane/
- Mission Planner 文档：https://ardupilot.org/planner/
- 参数列表：https://ardupilot.org/plane/docs/parameters.html

---

## 周总结（2026-01-26 填写）

### 完成情况
- [x] 《Mission Planner 功能手册》 ✅ 完成（约 900 行，6 章 + 附录）
- [x] SITL 任务完成 ✅ （TAKEOFF → 航点 → LOITER → LAND）
- [x] 截图/录屏保存 ✅ （保存至 `docs/03-Mission-Planner/20260123飞行记录/`）

### 主要收获
1. 掌握了 SITL 启动流程与 Mission Planner 连接方法
2. 深入学习了 Flight Data / Flight Plan / Initial Setup / Config/Tuning 四大界面
3. 理解了固定翼与多旋翼的差异（RTL 盘旋不降落、需要 LAND 进场航线）
4. 完成了完整的 TAKEOFF → 航点 → LOITER → LAND 航线任务
5. 学习了参数系统结构与 PID 参数位置

### 遇到的问题
1. SITL 路径配置错误（PATH 中旧路径优先）→ 已通过修改 ~/.profile 和 ~/.bashrc 解决
2. RTL 返航后只盘旋不降落 → 学习了固定翼需要 DO_LAND_START + 进场点 + LAND 的降落航线
3. 盘旋后绕出轨迹、高度偏差 → 理解了固定翼的物理特性和 TECS 高度控制

### 对下周的建议
- 第一周对参数系统已有初步了解，第二周可以深入学习
- 重点关注 PID 参数和 TECS 参数的实际影响
- 在 SITL 中尝试小幅调参，观察飞行行为变化

### 自我评价

| 项目 | 评分(1-5) |
|------|-----------|
| 产出质量 | ⭐⭐⭐⭐⭐ |
| 学习效率 | ⭐⭐⭐⭐ |
| 理解深度 | ⭐⭐⭐⭐ |

---

*第1周学习记录 - ✅ 已完成*
