# STM32F103 双轮自平衡小车

基于 STM32F103C8T6 的双轮自平衡小车项目，采用**串级 PID 控制**与**倒立摆动力学模型**，实现稳定自平衡与蓝牙遥控行走。

## 硬件平台

| 模块 | 型号 | 说明 |
|------|------|------|
| 主控 | STM32F103C8T6 | Cortex-M3, 72MHz, 64KB Flash, 20KB SRAM |
| 姿态传感器 | MPU6050 | 三轴陀螺仪 + 三轴加速度计, I2C 接口 |
| 电机驱动 | TB6612FNG | 双 H 桥, PWM 调速 |
| 电机 | 带编码器直流减速电机 | 双路编码器测速 |
| 蓝牙 | HC-05 / JDY-31 | UART 透传, 9600bps |
| 显示 | 0.96" OLED (SSD1306) | I2C 接口 (可选) |
| 电源 | 2S / 3S 锂电池 | 7.4V ~ 12.6V 输入 |

## 软件架构

```
main.c (超级循环)
  ├── App_Bat_Proc()       — 电池 ADC 采样 + LED 指示
  ├── App_Button_Proc()    — 按键启停状态机
  ├── App_Motor_Proc()     — 1ms 电机速度环 PI 控制
  ├── App_MPU6050_Proc()   — 5ms 姿态解算 (互补滤波)
  ├── App_Control_Proc()   — 5ms 串级平衡控制 (倒立摆模型)
  └── App_RC_Proc()        — 蓝牙遥控指令解析
```

- **裸机架构**，使用 `PERIODIC(T)` 宏实现协作式伪多任务调度
- **非 RTOS**，无操作系统依赖，适合初学者理解底层时序

## 控制算法

### 串级 PID 结构

采用**四环串级 PID** 控制，从外到内依次为：

```
遥控速度设定 → [速度环 PID] → θ_ref → [角度环 PID] → θ̇_ref → [角速度环 PID] → θ̈_ref
                              ↑                                  ↑
                          MPU6050 俯仰角                    MPU6050 角速度
```

**速度环** (最外层): 输入速度偏差，输出目标倾角 `θ_ref`，利用 `atan2(v_ref, g)` 进行倒立摆逆解算  
**角度环**: 输入角度偏差，输出目标角速度  
**角速度环** (最内层): 输入角速度偏差，输出角加速度 `θ̈`  
**转向环**: 独立 P 控制，以陀螺仪 Z 轴角速度 `gz` 为反馈，叠加差速到左右电机

### 倒立摆动力学模型

根据倒立摆模型将角加速度转化为轮胎线加速度：

```
ẍ = (g·sinθ - θ̈·lp) / cosθ
```

其中 `lp` 为轮轴到质心距离，`rw` 为轮胎半径。

### 互补滤波

对陀螺仪积分和加速度计解算进行融合：

```
angle = 0.95238 × angle_gyro + 0.04762 × angle_accel
```

对应截止频率约 200Hz 采样周期下的高通/低通滤波。

### 编码器测速

- 使用 EXTI 外部中断 + T 法测速
- 两路编码器独立计数，支持正反转判别

## 目录结构

```
├── user/                  # 应用层代码
│   ├── main.c             # 主函数 & 超级循环
│   ├── app_control.c/h    # 串级 PID 平衡控制
│   ├── app_motor.c/h      # 电机速度环 PI + PWM 输出
│   ├── app_mpu6050.c/h    # MPU6050 驱动 & 姿态解算
│   ├── app_encoder.c/h    # 编码器测速
│   ├── app_pid.c/h        # PID 控制器实现
│   ├── app_pwm.c/h        # TIM PWM 驱动 TB6612
│   ├── app_rc.c/h         # 蓝牙遥控协议解析
│   ├── app_bat.c/h        # 电池电压 ADC 采样
│   ├── app_button.c/h     # 按键状态机
│   └── app_usart2.c/h     # USART2 调试串口
├── my_lib/                # 底层驱动库
│   ├── si2c.c/h           # 软件 I2C (bit-bang)
│   ├── i2c.c/h            # 硬件 I2C 驱动
│   ├── spi.c/h            # SPI 驱动
│   ├── usart.c/h          # USART 驱动
│   ├── delay.c/h          # SysTick 延时 & 微秒时间戳
│   ├── pid.c/h            # PID 库 (通用版)
│   ├── qmath.c/h          # 三角函数查表实现
│   ├── button.c/h         # 按键驱动
│   ├── oled.c/h           # OLED 驱动 + 字体
│   ├── task.h             # 协作式伪多任务调度宏
│   └── str_cmd.c/h        # 字符串命令解析
├── test/                  # 模块单元测试
│   ├── bat_test.c/h       # 电池模块测试
│   ├── encoder_test.c/h   # 编码器测试
│   ├── mpu6050_test.c/h   # MPU6050 测试
│   ├── pid_test.c/h       # PID 测试
│   ├── pwm_test.c/h       # PWM 测试
│   └── qmath_test.c/h     # 三角函数测试
├── std_periph_driver/     # STM32 标准外设库
├── startup/               # 启动文件 (.s)
├── DebugConfig/           # 调试配置
├── banlance_car.uvprojx   # Keil MDK 工程文件
├── banlance_car.uvoptx    # Keil 工程选项
└── .clang-format          # 代码格式化规则
```

## 快速开始

### 环境要求

- **MDK-ARM (Keil uVision 5)** 或 **EIDE (Embedded IDE)**
- STM32F1xx DFP 芯片包
- ST-Link / J-Link 调试器

### 编译 & 烧录

1. 克隆仓库
   ```bash
   git clone https://github.com/<your-username>/STM32_Balance_Car.git
   ```

2. 用 Keil MDK 打开 `banlance_car.uvprojx`

3. 编译 (F7)，下载 (F8)

4. 或使用 EIDE (VS Code 插件):
   - 在 VS Code 中打开项目文件夹
   - 安装 Embedded IDE 插件
   - 导入 Keil 工程或直接编译

### 硬件接线

| 外设 | STM32 引脚 | 说明 |
|------|-----------|------|
| MPU6050 SCL | PA5 | 软件 I2C |
| MPU6050 SDA | PA6 | 软件 I2C |
| TB6612 AIN1/2 | PB12/PB13 | 左电机方向 |
| TB6612 BIN1/2 | PB14/PB15 | 右电机方向 |
| TB6612 PWMA | PA1 (TIM2-CH2) | 左电机 PWM |
| TB6612 PWMB | PB7 (TIM4-CH2) | 右电机 PWM |
| 编码器 L | PB3 (EXTI3) | 左编码器 A 相 |
| 编码器 R | PB4 (EXTI4) | 右编码器 A 相 |
| 蓝牙 TX/RX | PB10/PB11 | USART3 |
| 调试串口 | PA2/PA3 | USART2, 921600bps |
| 电池 ADC | PA0 (ADC1-IN0) | 分压检测 |
| 按键 | PA4 | 启停控制 |

## 遥控协议

蓝牙串口 (USART3, 9600bps) 接收 ASCII 文本指令：

```
move <turnSpeed> <moveSpeed>\n
```

| 参数 | 范围 | 含义 |
|------|------|------|
| `turnSpeed` | -100 ~ +100 | 转向速度，正值为右转 |
| `moveSpeed` | -100 ~ +100 | 前进速度，正值为前进 |

示例:
- `move 0 50\n` — 以 50% 速度前进
- `move 30 0\n` — 原地右转
- `move 0 0\n` — 停车

> **注意**: 协议无校验位，无线传输距离取决于蓝牙模块（通常 10m 以内）。

## 按键操作

- **单击**: 启动 / 停止平衡车
- 启动后自动进入平衡模式，蓝牙指令可覆盖遥控

## 已知问题 & TODO

- [ ] 硬件 I2C 未调通，暂用软件 I2C (时序精度受限)
- [ ] 编码器测速使用 EXTI 而非定时器编码器模式 (资源利用不优)
- [ ] 未使用 DMA，串口收发为阻塞/中断模式
- [ ] 缺少 IWDG 独立看门狗和倾覆保护
- [ ] 互补滤波 dt 为固定值，实际随主循环负载漂移
- [ ] NVIC 优先级分组为 Group_0 (无抢占优先级)
- [ ] 速度环/角度环未做 anti-windup

欢迎提交 Issue 和 PR 改进！

## 参考资料

- [MPU6050 数据手册](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [STM32F103 参考手册 (RM0008)](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
- [TB6612FNG 数据手册](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660)
- 铁头山羊 — STM32 平衡车教程

## 许可证

本项目采用 [MIT License](LICENSE) 开源。

---

**作者**: YouZi  
**联系**: youziuser@hrbeu.edu.cn
