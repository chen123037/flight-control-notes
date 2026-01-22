### 主题：固定翼 Roll/Pitch/Yaw PID 基础与默认值逻辑

【一句话定义】  
Roll/Pitch/Yaw 的 PID 参数用于稳定三轴姿态，兼顾响应速度与抖动抑制。

【核心概念】  
- Roll/Pitch 为标准 PID：P/I/D + 积分上限  
- Yaw 常用“偏航混滚转”，积分与阻尼可关闭  
- INT_MAX 用于限制积分饱和

【关键公式/逻辑】  
- P：误差越大输出越大，响应更快  
- I：消除长期稳态误差  
- D：抑制快速变化与震荡  
- INT_MAX：限制积分项最大值  

【默认值解读（示例）】  
- Roll 更激进：P/I/D 较大，响应更快  
- Pitch 更保守：P/I/D 较小，减少俯仰过冲  
- Yaw 以混滚转为主：Yaw-to-Roll=1，积分与阻尼为 0  

【常见误区】  
- 盲目增大 P/I 导致震荡或积分饱和  
- 把 Yaw 当作完整 PID 环处理而忽略“混滚转”逻辑  

【最小示例】  
Roll：P 0.3 / I 0.25 / D 0.01743 / INT_MAX 0.00666  
Pitch：P 0.15 / I 0.11 / D 0.007265 / INT_MAX 0.00666  
Yaw：Yaw-to-Roll 1 / I 0 / 抑制 0 / 积分上限 15

【关联笔记】  
- `../03-Mission-Planner/功能手册.md`
