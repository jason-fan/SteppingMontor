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

基于 STM8S103F3P6（TSSOP20）的两相四线步进电机控制工程。采用 SDCC 编译，全部驱动以「结构体 + 函数指针」的面向对象风格自研实现，不依赖官方库与第三方代码。支持单键双模式控制（连续转 / 点动）、双击换向、电位器连续调速、双 LED 状态指示、硬件 BEEP 提示音与 UART 调试输出。

## 硬件接线（引脚分配，TSSOP20）

| 功能 | 引脚 | 说明 |
|---|---|---|
| 单按键 | PD3（pin20） | 按键接 GND，内部上拉，下降沿触发 EXTI3；单击/双击/长按见功能说明 |
| 电机 **A-1A**（代码 IN1） | PC3 | 接 L9110S 丝印 **A-1A**（推挽输出） |
| 电机 **A-1B**（代码 IN3） | PC5 | 接 L9110S 丝印 **A-1B**（推挽输出） |
| 电机 **B-1B**（代码 IN2） | PC7 | 接 L9110S 丝印 **B-1B**（推挽输出） |
| 电机 **B-1A**（代码 IN4） | PC6 | 接 L9110S 丝印 **B-1A**（推挽输出） |
| 电位器 | PC4（AIN2） | 10k 电位器中心抽头，两端接 3.3V / GND，ADC 单次转换 |
| 模式/运行指示灯 | PA3 | 高电平点亮；**连续转模式**下随电机亮灭（转动亮、停止灭），**点动模式**下以 250ms 半周期持续闪烁作为模式标记，点动单击不改变该灯；**上电默认熄灭** |
| 反转指示灯 | PB4（pin12） | **真开漏引脚，低电平点亮**（LED 阳极经限流电阻接 VDD、阴极接 PB4）；反转（CCW）时亮，正转（CW）时灭；**上电默认熄灭** |
| 蜂鸣器 | PD4 | STM8S 硬件 BEEP 外设输出（pin1），可选 1/2/4 kHz |
| UART TX | PD5 | 115200 8N1 调试输出（printf） |
| UART RX | PD6 | 115200 8N1 调试输入（主循环回显） |

> 电机四个绕组输出为普通推挽 GPIO，**严禁直接驱动电机线圈**，必须经 L9110s/ULN2003 驱动板。

## 功能说明

### 按键（PD3，非阻塞 FSM）

按键行为按**运行模式**区分，上电默认「连续转」，长按切换模式：

- **单击**
  - **连续转模式下**：电机 启动 ↔ 停止 切换。启动：PA3 亮、BEEP 2kHz/100ms、串口 `MOTOR START`；停止：PA3 灭、BEEP 1kHz/100ms、串口 `MOTOR STOP`
  - **点动模式下**：走一步（按当前方向前进一个半步，相位立即通电 `on_ms` 后自动断电），BEEP 1kHz/30ms、串口 `JOG STEP`；**单击不影响 PA3**，PA3 保持点动模式闪烁节奏
- **双击**：正转（CW）↔ 反转（CCW）切换。PB4 反转灯 CCW 亮 / CW 灭，BEEP 4kHz/80ms，串口 `DIR CCW` / `DIR CW`
- **长按**：连续转 ↔ 点动 模式切换。切换时**先立即停机并熄灭 PA3**，避免连续运转状态串入点动模式；进入**点动模式后 PA3 转为 250ms 半周期闪烁**（主循环用 `g_tick` 计时），回到连续转则恢复为随电机亮灭的运行灯；BEEP 4kHz/120ms 进入点动、2kHz/120ms 回到连续转；串口 `MODE JOG` / `MODE CONTINUOUS`

### 电位器调速（PC4 = AIN2）

- 每 200ms 采样一次，ADC 值 0~1023 线性映射到步进周期：
  `period = 50 - (raw * 44) / 1023`（代码宏 `MOTOR_PERIOD_MAX=50U`、`MOTOR_PERIOD_MIN=6U`）
- **ADC=0（逆时针到头）→ 50ms/步（最慢）；ADC=1023（顺时针到头）→ 6ms/步（最快）**
- 速度发生变化的采样点才更新电机并打印 `pot=<raw> spd=<period> ms/step`，避免刷屏
- ADC 驱动：单次转换 + 每读 8 次平均滤波 + EOC 超时保护（超时返回 0），读前带输入稳定延时

### 电机驱动（1-2 相励磁半步）

- 8 拍相位表：CW 沿表正向换相，CCW 反向换相
- `setSpeed` 将周期钳制在 **[1, 250] ms/步** 区间
- **每步通电时间窗**：每步开始时输出目标相序并通电 `on_ms`（经验值 **4ms**）毫秒；on_ms 时间到且尚未到下一步时，四路输出全部清零（L9110S 输入全 0，线圈断电），到 `period_ms` 时刻执行下一步换相并重新通电——慢速下线圈只在通电窗口内有电流，**省电流、电机不发烫**
- `on_ms` 由 `Motor_Create(ports, pins, on_ms)` 传入（`motor.h` 中 `MOTOR_ON_MS` 为推荐经验值 4U）；取值应 ≤ 最小 period（本项目调速下限 6ms/步，全档位 `period ≥ 6ms > on_ms=4ms`，因此**每个档位都保留断电窗口**：最快档 4ms 通 / 2ms 断，最慢档 4ms 通 / 46ms 断）；若 on_ms ≥ 当前 period，断电分支不触发，等效**常通电**（旧行为，本项目已不存在该情况）
- 停止时释放全部线圈（输出低电平），避免长时间通电发热
- **单步点动 `stepOnce()`**：按当前方向换相一个半步并通电，`running` 保持 0；`motor.onTime()` 在 `on_ms` 到期后自动断电（`jog_active` 标志 + `jog_acc` 计数），进入连续转（`start`）或停止（`stop`）时该标志被清除

### L9110S 驱动板接线线序

- **MCU → L9110S 输入（按驱动板丝印）**：**PC3 → A-1A**、**PC5 → A-1B**、**PC6 → B-1A**、**PC7 → B-1B**
- **代码编号对照**（仅用于阅读源码，不作接线依据）：IN1=PC3、IN2=PC7、IN3=PC5、IN4=PC6
- **绕组配对**：**A 相 = PC3(A+) + PC5(A−)**，**B 相 = PC6(B−) + PC7(B+)**；即同一绕组的两根线必须接到驱动板的同一路通道（A-1A/A-1B 为一组，B-1A/B-1B 为一组），不可把同一绕组的两线跨组连接
- **L9110S → 步进电机**：**A 相两线接 A-OA / A-OB**，**B 相两线接 B-OA / B-OB**
- **供电**：驱动板 **VM 接 5V**，**GND 与 MCU 共地**；电机按额定电压接到驱动板输出
- **相位表位序说明（重要）**：`motor.c` 头部注释的 8 拍表按 `bit3=IN1、bit2=IN2、bit1=IN3、bit0=IN4` 的位序书写，而 `motor_write_phase()` 实际按 `bit0=IN1、bit1=IN2、bit2=IN3、bit3=IN4` 逐位写引脚（`pattern & (1<<i) → pin[i]`）。两者**互为镜像**，故 `phase_table` 的实际通电相序是注释序列的逆序，**`dir=0` 时物理转向与注释标称的 CW 相反**。这是已确认的镜像关系，不是接线错误
- **转向调整**：若电机转向与预期相反，**交换任一组绕组的两根线**（A-OA/A-OB 或 B-OA/B-OB 二者之一）即可，无需改代码

### 状态指示与提示音

- **上电默认双灯熄灭**：`led.init` 内部会把输出置高点亮，main 中 init 后显式 `led_run.off` + `led_dir.off` 使其默灭，仅在动作时点亮
- 上电自检：BEEP 单次 2kHz/100ms；按键提示音使用 `beep.once(档位, 毫秒)`，第二参数传 **1/2/4 档位**（1k/2k/4kHz），非频率数值

### UART 调试（PD5 TX / PD6 RX）

- 115200 8N1；发送走 128 字节非阻塞环形缓冲，由主循环 `uart.tx_service()` 排空（按键回调运行在 TIM1 中断内，仅入队不阻塞）
- RXNE 中断接收，主循环回显收到的字节
- 打印内容：上电 `STM8S103 STEPPER ready`；按键事件 `MOTOR START/STOP`（连续转）、`JOG STEP`（点动一步）、`DIR CW/CCW`、`MODE JOG/CONTINUOUS`；调速 `pot=... spd=... ms/step`

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
| motor.h / motor.c | 步进电机对象：1-2 相半步换相、启停、单步点动（stepOnce）、正反转、调速钳制 |
| key.h / key.c | 按键对象：任意端口 EXTI 参数化注册，非阻塞单击/双击/长按 FSM |
| adc.h / adc.c | ADC1 对象：PC4(AIN2) 单次转换 + 平均滤波 + EOC 超时 |
| uart.h / uart.c | UART1 对象：115200 8N1，非阻塞 TX 环形缓冲 + RX 中断 |
| led.h / led.c | LED 对象：启停/反转指示灯（active_high 可配） |
| beep.h / beep.c | 硬件 BEEP 对象：PD4，1/2/4 kHz 定时发声（LSI 时钟） |
| build.bat | Windows 编译脚本（SDCC + `--std-c99`） |
| Makefile | Make 构建脚本（含烧录目标） |

## 注意事项

- **引脚约束**：STM8S103F3P6 为 TSSOP20，仅引出部分 GPIO；本项目使用的 PA3/PB4/PC3/PC4/PC5/PC6/PC7/PD3/PD4/PD5/PD6 均为实际可用引脚（**PB3 在 TSSOP20 上未引出，不可使用**；PB4/PB5 为真开漏，只能低电平驱动）。PA4~PA7 等未引出引脚**不可配置为输出使用**，扩展功能前务必对照数据手册确认
- **调速范围**：电位器映射为 **6~50ms/步**（代码 `MOTOR_PERIOD_MIN=6U`、`MOTOR_PERIOD_MAX=50U`）。更快的 1~2ms/步（500~1000 半步/秒）远超 5V 小步进电机的牵入频率，实测出现**失步、堵转**，且周期低于 `on_ms`（4ms）会让最快档变成常通电、失去省电效果，故下限取 6ms/步。**实测数据**：最快档（6ms/步）线圈电流约 **0.4A**，最慢档（50ms/步）约 **0.02A**，连续运行**电机不发热**。如更换电机/驱动板需调 `main.c` 中 `MOTOR_PERIOD_MIN/MAX` 宏
- **驱动板必需**：绕组信号为 GPIO 电平，不可直连电机线圈，必须接 L9110s/ULN2003 驱动板
- **编译选项**：必须使用 `--std-c99`，否则 UART 的 `printf` 变参调用（`%u` 等）与代码声明会出问题；推荐直接运行 `build.bat`
- **BEEP 档位参数**：`beep.once(&beep, khz, ms)` 的 `khz` 传档位 **1 / 2 / 4**，不是任意频率值
- **LED 上电状态**：上电两灯默认熄灭（`main` 中 init 后显式 off），如需常亮请调整 `main.c` 初始化顺序
- **UART 输出**：发送为环形缓冲非阻塞设计，主循环必须持续调用 `uart.tx_service()` 排空，否则输出滞留
- **低功耗唤醒**：主循环使用 `wfi` 而非 `halt`（TIM1 更新中断无法唤醒 halt）
- **电源与电平**：电位器参考 3.3V（与 MCU 同电源域）；驱动板电源按电机额定电压独立供电
- **通电时间窗**：`on_ms` 由 `Motor_Create` 传入，`motor.h` 中 `MOTOR_ON_MS=4U` 为经验默认值；当前最快档 6ms/步 > 4ms，全速域都能形成断电窗口（最快档 4ms 通 / 2ms 断）；若把 `MOTOR_PERIOD_MIN` 调到 ≤4U，最快档将等效常通电（属预期行为，但会失去省电效果）
*（内容由AI生成，仅供参考）*
