---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 495757a73352da7aae1b306638d4f5fe_eb74b351d4f211f180d5525400248c00
    ReservedCode1: mr/TTUDzqhzkve/kCYDOTLgtfRjB2bLh4EGhJAsvMH7m1qAjv2pDnVWgwa/jKVBUtLPLmID40k/tabvIT1ZRLqF4u2NPhGRjcKSANaz1YlloNTv452W0iyvETqGrtYCuruPCzPLf9JmaQtO+Tg0uO/RCSVJJzroot0LyuuTfvsZC8cE6vpAKV0O8ADM=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 495757a73352da7aae1b306638d4f5fe_eb74b351d4f211f180d5525400248c00
    ReservedCode2: mr/TTUDzqhzkve/kCYDOTLgtfRjB2bLh4EGhJAsvMH7m1qAjv2pDnVWgwa/jKVBUtLPLmID40k/tabvIT1ZRLqF4u2NPhGRjcKSANaz1YlloNTv452W0iyvETqGrtYCuruPCzPLf9JmaQtO+Tg0uO/RCSVJJzroot0LyuuTfvsZC8cE6vpAKV0O8ADM=
---





# project2 - STM8S103F3P6 两相四线步进电机控制

基于 STM8S103F3P6（TSSOP20）的两相四线步进电机控制工程。采用 SDCC 编译，全部驱动以「结构体 + 函数指针」的面向对象风格自研实现，不依赖官方库与第三方代码。支持单键启停/换向、电位器连续调速、双 LED 状态指示、硬件 BEEP 提示音与 UART 调试输出。

## 硬件接线（引脚分配，TSSOP20）

| 功能 | 引脚 | 说明 |
|---|---|---|
| 单按键 | PD3（pin2） | 按键接 GND，内部上拉，下降沿触发 EXTI3；单击/双击/长按见功能说明 |
| 电机 IN1 | PC3 | 接 L9110s/ULN2003 驱动板 IN1（推挽输出） |
| 电机 IN2 | PC7 | 接 L9110s/ULN2003 驱动板 IN2（推挽输出） |
| 电机 IN3 | PC5 | 接 L9110s/ULN2003 驱动板 IN3（推挽输出） |
| 电机 IN4 | PC6 | 接 L9110s/ULN2003 驱动板 IN4（推挽输出） |
| 电位器 | PC4（AIN2） | 10k 电位器中心抽头，两端接 3.3V / GND，ADC 单次转换 |
| 启停指示灯 | PA3 | 高电平点亮；电机运行时亮，停止时灭；**上电默认熄灭** |
| 反转指示灯 | PB3 | 高电平点亮；反转（CCW）时亮，正转（CW）时灭；**上电默认熄灭** |
| 蜂鸣器 | PD4 | STM8S 硬件 BEEP 外设输出（pin1），可选 1/2/4 kHz |
| UART TX | PD5 | 115200 8N1 调试输出（printf） |
| UART RX | PD6 | 115200 8N1 调试输入（主循环回显） |

> 电机四个绕组输出为普通推挽 GPIO，**严禁直接驱动电机线圈**，必须经 L9110s/ULN2003 驱动板。

## 功能说明

### 按键（PD3，非阻塞 FSM）

- **单击**：电机 启动 ↔ 停止 切换。启动：PA3 亮、BEEP 2kHz/100ms、串口 `MOTOR START`；停止：PA3 灭、BEEP 1kHz/100ms、串口 `MOTOR STOP`
- **双击**：强制停止电机（PA3 灭、串口 `MOTOR STOP`、BEEP 1kHz/100ms）
- **长按**：正转（CW）↔ 反转（CCW）切换。PB3 反转灯 CCW 亮 / CW 灭，BEEP 4kHz/80ms，串口 `DIR CCW` / `DIR CW`

### 电位器调速（PC4 = AIN2）

- 每 200ms 采样一次，ADC 值 0~1023 线性映射到步进周期：
  `period = 50 - (raw * 48) / 1023`（代码宏 `MOTOR_PERIOD_MAX=50U`、`MOTOR_PERIOD_MIN=2U`）
- **ADC=0（逆时针到头）→ 50ms/步（最慢）；ADC=1023（顺时针到头）→ 2ms/步（最快）**
- 速度发生变化的采样点才更新电机并打印 `pot=<raw> spd=<period> ms/step`，避免刷屏
- ADC 驱动：单次转换 + 每读 8 次平均滤波 + EOC 超时保护（超时返回 0），读前带输入稳定延时

### 电机驱动（1-2 相励磁半步）

- 8 拍相位表：CW 沿表正向换相，CCW 反向换相
- `setSpeed` 将周期钳制在 **[1, 250] ms/步** 区间
- **每步通电时间窗**：每步开始时输出目标相序并通电 `on_ms`（经验值 **4ms**）毫秒；on_ms 时间到且尚未到下一步时，四路输出全部清零（L9110S 输入全 0，线圈断电），到 `period_ms` 时刻执行下一步换相并重新通电——慢速下线圈只在通电窗口内有电流，**省电流、电机不发烫**
- `on_ms` 由 `Motor_Create(ports, pins, on_ms)` 传入（`motor.h` 中 `MOTOR_ON_MS` 为推荐经验值 4U）；取值应 ≤ 最小 period（本项目调速下限 2ms/步）；若 on_ms ≥ 当前 period，断电分支不触发，等效**常通电**（旧行为）
- 停止时释放全部线圈（输出低电平），避免长时间通电发热

### L9110S 驱动板接线线序

- **MCU → L9110S 输入**：**PC3 → A-1A**、**PC5 → A-1B**、**PC6 → B-1A**、**PC7 → B-1B**
- **L9110S → 步进电机**：**A 相两线接 A-OA / A-OB**，**B 相两线接 B-OA / B-OB**
- **供电**：驱动板 **VM 接 5V**，**GND 与 MCU 共地**；电机按额定电压接到驱动板输出
- **转向**：若电机转向与预期相反，**对调 A 相两线（A-OA / A-OB）或 B 相两线（B-OA / B-OB）之一**即可，无需改代码

### 状态指示与提示音

- **上电默认双灯熄灭**：`led.init` 内部会把输出置高点亮，main 中 init 后显式 `led_run.off` + `led_dir.off` 使其默灭，仅在动作时点亮
- 上电自检：BEEP 单次 2kHz/100ms；按键提示音使用 `beep.once(档位, 毫秒)`，第二参数传 **1/2/4 档位**（1k/2k/4kHz），非频率数值

### UART 调试（PD5 TX / PD6 RX）

- 115200 8N1；发送走 128 字节非阻塞环形缓冲，由主循环 `uart.tx_service()` 排空（按键回调运行在 TIM1 中断内，仅入队不阻塞）
- RXNE 中断接收，主循环回显收到的字节
- 打印内容：上电 `STM8S103 STEPPER ready`；按键事件 `MOTOR START/STOP`、`DIR CW/CCW`；调速 `pot=... spd=... ms/step`

### 运行机制

- 时钟：HSI 16MHz（`CLK_CKDIVR=0x00`）
- 时间片：TIM1 1ms 更新中断驱动 `key.onTime` / `motor.onTime` / `beep.update`（提示音自动关闭）
- 主循环：`wfi` 等待中断唤醒（注释明确禁用 `halt`，因 TIM1 无法唤醒 halt），唤醒后先排空 UART 再按 200ms 节拍采样电位器

## 编译与烧录

在 cmd 中执行（避免 PowerShell 重定向编码问题）：

```bat
cd /d D:\stm8s_project\project2
build.bat
rem 或 make -f Makefile

rem 烧录（stm8flash，ST-Link v2）
D:\sdcc\bin\stm8flash.exe -c stlinkv2 -p stm8s103f3 -w project2.ihx
```

- `build.bat`：SDCC 命令行编译全部源文件 → `project2.ihx`，编译选项**必须包含 `--std-c99`**（否则 UART printf 变参与声明会乱码/报错），生成后自动 packihx 并清理中间文件
- `Makefile`：`make -f Makefile` 可编译；提供 `all / hex / size / flash / flash_uart / clean / help` 等目标（`flash_uart` 目标依赖串口参数，详见 Makefile 内注释）

## 文件清单

| 文件 | 说明 |
|---|---|
| main.c | 应用编排 + 时钟/TIM1/EXTI/UART1_RX 中断向量表（含 main 单元向量表） |
| motor.h / motor.c | 步进电机对象：1-2 相半步换相、启停、正反转、调速钳制 |
| key.h / key.c | 按键对象：任意端口 EXTI 参数化注册，非阻塞单击/双击/长按 FSM |
| adc.h / adc.c | ADC1 对象：PC4(AIN2) 单次转换 + 平均滤波 + EOC 超时 |
| uart.h / uart.c | UART1 对象：115200 8N1，非阻塞 TX 环形缓冲 + RX 中断 |
| led.h / led.c | LED 对象：启停/反转指示灯（active_high 可配） |
| beep.h / beep.c | 硬件 BEEP 对象：PD4，1/2/4 kHz 定时发声（LSI 时钟） |
| build.bat | Windows 编译脚本（SDCC + `--std-c99`） |
| Makefile | Make 构建脚本（含烧录目标） |

## 注意事项

- **引脚约束**：STM8S103F3P6 为 TSSOP20，仅引出部分 GPIO；本项目使用的 PA3/PB3/PC3/PC4/PC5/PC6/PC7/PD3/PD4/PD5/PD6 均为实际可用引脚。PA4~PA7 等未引出引脚**不可配置为输出使用**，扩展功能前务必对照数据手册确认
- **调速范围**：电位器映射为 **2~50ms/步**（代码 `MIN=2U` 防护）。曾验证 1ms/步时 ULN2003 驱动的小电机会失步堵转，故最小值限为 2ms/步；如更换电机/驱动板需调 `main.c` 中 `MOTOR_PERIOD_MIN/MAX` 宏
- **驱动板必需**：绕组信号为 GPIO 电平，不可直连电机线圈，必须接 L9110s/ULN2003 驱动板
- **编译选项**：必须使用 `--std-c99`，否则 UART 的 `printf` 变参调用（`%u` 等）与代码声明会出问题；推荐直接运行 `build.bat`
- **BEEP 档位参数**：`beep.once(&beep, khz, ms)` 的 `khz` 传档位 **1 / 2 / 4**，不是任意频率值
- **LED 上电状态**：上电两灯默认熄灭（`main` 中 init 后显式 off），如需常亮请调整 `main.c` 初始化顺序
- **UART 输出**：发送为环形缓冲非阻塞设计，主循环必须持续调用 `uart.tx_service()` 排空，否则输出滞留
- **低功耗唤醒**：主循环使用 `wfi` 而非 `halt`（TIM1 更新中断无法唤醒 halt）
- **电源与电平**：电位器参考 3.3V（与 MCU 同电源域）；驱动板电源按电机额定电压独立供电
- **通电时间窗**：`on_ms` 由 `Motor_Create` 传入，`motor.h` 中 `MOTOR_ON_MS=4U` 为经验默认值；若需要在最快档（2ms/步）也断电，应将其调至 ≤2ms；on_ms ≥ period 时等效常通电，属预期行为
*（内容由AI生成，仅供参考）*
