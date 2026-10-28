---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 495757a73352da7aae1b306638d4f5fe_3abf10c2d29e11f18460525400393706
    ReservedCode1: PkJi/IgqDFauIcWVWS3b9g47TGonZviR/49mhxKfOH5kRaaL4VzcKQzZo1iYCV4Nwv7M2SrR0XXtGWuoHzmd9jCabeN+x+HTVHHALv+zoCWgMDkeKa30TihmITEzbhl8vWWddfyW+FlOafym+4rD13k0B+iA9qxVAJ6aS0LbkhJ0a/sc2CCnmTNv4X0=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 495757a73352da7aae1b306638d4f5fe_3abf10c2d29e11f18460525400393706
    ReservedCode2: PkJi/IgqDFauIcWVWS3b9g47TGonZviR/49mhxKfOH5kRaaL4VzcKQzZo1iYCV4Nwv7M2SrR0XXtGWuoHzmd9jCabeN+x+HTVHHALv+zoCWgMDkeKa30TihmITEzbhl8vWWddfyW+FlOafym+4rD13k0B+iA9qxVAJ6aS0LbkhJ0a/sc2CCnmTNv4X0=
---



# project2 - STM8S103F3P6 两相四线步进电机控制

在 D:\stm8s_project\project1 基础上，以相同风格（SDCC + 面向对象结构体/函数指针、无官方库、无第三方依赖）新建的步进电机控制工程。

## 硬件接线（TSSOP20）

| 功能 | 引脚 | 说明 |
|---|---|---|
| 单按键 | PD3 → GND（pin2） | 单击：电机启动/停止切换；长按：正反转切换（内部上拉） |
| 电机 IN1 | PB4 | 接 L9110s/ULN2003 驱动板 IN1 |
| 电机 IN2 | PB5 | 接 L9110s/ULN2003 驱动板 IN2 |
| 电机 IN3 | PC5 | 接 L9110s/ULN2003 驱动板 IN3 |
| 电机 IN4 | PC6 | 接 L9110s/ULN2003 驱动板 IN4 |
| 电位器 | PC4 (AIN2) | 10k 电位器中心抽头，两端接 3.3V/GND |
| 启停指示灯 | PA3 | 电机运行时点亮（默认高电平亮，按开发板调整）；**上电默认熄灭** |
| 反转指示灯 | PB3 | 反转（CCW）时点亮，正转（CW）熄灭（PB3 空闲引脚）；**上电默认熄灭** |
| 蜂鸣器 | PD4 | STM8S 硬件 BEEP 输出（1/2/4 kHz，按键提示音） |
| UART TX | PD5 | 115200 8N1 调试输出 |
| UART RX | PD6 | 115200 8N1 调试输入（回显） |

> 电机绕组请勿直接接 MCU 引脚，必须经 L9110s/ULN2003 驱动板。

## 功能

- 单键控制（PD3，复用 project1 key.c 的单击/长按非阻塞 FSM）：
  - 单击：电机启动 ↔ 停止切换，PA3 启停指示灯同步亮/灭，蜂鸣器提示音
  - 长按：正转（CW）↔ 反转（CCW）切换，PB3 反转指示灯反转时点亮
- 按键提示音：单击启动 2kHz/100ms、单击停止 1kHz/100ms、长按换向 4kHz/80ms（硬件 BEEP 外设 PD4）
- 电位器调速：ADC 值 0~1023 线性映射为 50ms/步（最慢）~ 2ms/步（最快），每 200ms 采样一次
- 换相时序：1-2 相励磁（半步）8 拍相位表，CW 正向走表、CCW 反向走表，停止时释放全部线圈
- 面向对象封装：Motor / KEY / ADC / UART / LED / BEEP 均为结构体 + 函数指针绑定方法

## 编译烧录（cmd，避免 PowerShell 重定向编码问题）

```bat
cd /d D:\stm8s_project\project2
build.bat
rem 或 make -f Makefile

rem 烧录
D:\sdcc\bin\stm8flash.exe -c stlinkv2 -p stm8s103f3 -w project2.ihx
```

## 文件清单

| 文件 | 说明 |
|---|---|
| main.c | 应用编排 + 中断向量表（TIM1/EXTI/UART1_RX） |
| motor.h / motor.c | 步进电机对象：换相时序、启停、正反转、调速 |
| key.h / key.c | 按键对象（复用 project1 实现）：非阻塞单击/长按 FSM + EXTI |
| adc.h / adc.c | ADC 对象（复用 project1 实现）：PC4 电位器采样 |
| uart.h / uart.c | UART 对象（复用 project1 实现）：115200 调试串口 |
| led.h / led.c | LED 对象（复用 project1 实现）：启停/反转指示灯 |
| beep.h / beep.c | BEEP 对象（移植 project1 实现）：硬件 BEEP 外设 PD4，按键发声 |
| build.bat | Windows 编译脚本 |
| Makefile | Makefile 构建脚本 |
*（内容由AI生成，仅供参考）*
*（内容由AI生成，仅供参考）*
*（内容由AI生成，仅供参考）*
