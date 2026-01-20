# Console / Map / ArduPlane 窗口说明

本说明基于 `docs/04-ArduPlane参数` 目录下的三张截图：
- `Console界面.png`
- `Map界面.png`
- `ArduPlane界面.png`

---

## 1) Console 窗口（Console界面.png）

### 顶部状态条
- `MANUAL`：当前飞行模式为手动
- `ARM`：飞控已解锁
- `GPS: OK6 (10)`：GPS 状态正常，括号内为卫星数量
- `Vcc 5.00`：供电电压
- `Radio: --`：未检测到遥控信号
- `INS / MAG / AS / RNG / AHRS / EKF / LOG / FEN`：传感器/估计模块状态（绿色为正常）

### 关键飞行数据
- `Hdg 352/230`：航向信息（当前/目标或估计值，取决于模式）
- `Alt 0m`：相对高度
- `AGL 0m/0m`：离地高度（地形高度相关）
- `AirSpeed 0m/s`：空速
- `GPSSpeed 0m/s`：地速
- `Thr 0`：油门
- `Roll 0`、`Pitch 0`：滚转角/俯仰角
- `Wind -180/0m/s`：风向 180°、风速 0

### 任务信息行
- `WP 0`：当前航点索引
- `Distance / Bearing`：到航点距离与方位（未执行任务时为空）
- `AltError / AspdError`：高度/空速误差
- `FlightTime`：飞行时间
- `ETR`：预计剩余时间
- `Param 1412/1412`：参数读取完成数

### 日志区域
- `Got COMMAND_ACK: ... ACCEPTED`：飞控已接受地面站指令
- `Flight battery 100 percent`：电池电量模拟值

---

## 2) Map 窗口（Map界面.png）

### 顶部文字信息
- `Cursor: ...`：鼠标当前位置经纬度/高度
- `Click: ...`：点击位置经纬度/高度
- `Distance: ...`：当前位置到点击点的距离
- `Bearing: ...`：当前位置到点击点的方位角
- `height: ...`：地形高度（地图数据）

### 地图显示
- 黄色网格：地图网格/测距辅助
- 飞机图标：当前飞机位置与朝向

---

## 3) ArduPlane 窗口（ArduPlane界面.png）

### 启动与端口信息
- `Setting SIM_SPEEDUP=...`：仿真倍率
- `Starting SITL input`：开始仿真输入
- `bind port 5760 for SERIAL0` / `SERIAL0 on TCP port 5760`：串口映射
- `bind port 5762 ...` / `SERIAL1 ...`：第二路串口映射
- `bind port 5763 ...` / `SERIAL2 ...`：第三路串口映射

### 连接与参数加载
- `Waiting for connection ...`：等待地面站连接
- `Connection on serial port 5760`：Mission Planner 已连接
- `Loaded defaults from ... plane.parm`：加载默认参数文件

### HOME 信息
- `Home: ... alt=... hdg=...`：HOME 点坐标与高度、初始航向

---

## 4) 关于高度差的说明

截图里 HOME 高度显示有差异（例如 584m 与 587m），常见原因：
- 地形高度数据源不同（SRTM/地图高度 vs 仿真高度）
- 显示刷新时差或四舍五入
