# 【2026-复旦大学-星云EGA】 云台yaw轴滑模控制器

# 使用说明
## 在你的主逻辑中：
### 初始化SMC类
`SMC YawSMC(20, 120, 0, 0.001, 21, 27, 25000, 0.8, 0.5);`
****
## 在你的主循环中：
### 设置滑模控制器目标值
`YawSMC.target = 你的角度目标;`
### 调用SMC计算器
`YawSMC.SMC_Tick(angle_now,angle_vel);`  
注意角度为°，角速度为°/s
### 最后，输出电流
`YourCanSendFunction(YawSMC.u);`
### tips:可能会存在正反馈的问题，建议检查反馈符号和输出电流的符号
****
by 李欣睿,任轩呈,梅峻炜