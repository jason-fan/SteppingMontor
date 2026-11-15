# project2 - STM8S103F3P6 两相四线步进电机控制

基于 STM8S103F3P6（TSSOP20）的两相四线步进电机控制工程。采用 SDCC 编译，全部驱动以「结构体 + 函数指针」的面向对象风格自研实现，不依赖官方库与第三方代码。支持单键双模式控制（连续转 / 点动）、双击换向、电位器连续调速、双 LED 状态指示、硬件 BEEP 提示音与 UART 调试输出。

## 硬件接线（引脚分配，TSSOP20）

| 功能 | 引脚 | 说明 |
|---|---|---|
| 单按键 | PD3（pin20） | 按键接 GND，内部上拉，下降沿触发 EXTI3；单击/双击/长按见功能说明 |
| 电机 A-1A（代码 IN1） | PC3（pin13） | 接 L9110S 丝印 A-1A，推挽输出 |
| 电机 A-1B（代码 IN3） | PC5（pin15） | 接 L9110S 丝印 A-1B，推挽输出 |
| 电机 B-1B（代码 IN2） | PC7（pin17） | 接 L9110S 丝印 B-1B，推挽输出 |
| 电机 B-1A（代码 IN4） | PC6（pin16） | 接 L9110S 丝印 B-1A，推挽输出 |
| 电位器 | PC4（pin14，AIN2） | 10k 电位器中心抽头，两端接 3.3V / GND，ADC 单次转换 + 平均滤波 |
| 模式/运行指示灯 | PA3（pin10） | 高电平点亮；连续转模式下随电机亮灭（转动亮、停止灭），点动模式下以 250ms 半周期持续闪烁作为模式标记，点动单击不改变该灯；上电默认熄灭 |
| 反转指示灯 | PB4（pin12） | 真开漏引脚，低电平点亮（LED 阳极经限流电阻接 VDD、阴极接 PB4）；反转（CCW）时亮，正转（CW）时灭；上电默认熄灭 |
| 蜂鸣器 | PD4（pin1） | STM8S 硬件 BEEP 外设输出，可选 1 / 2 / 4 kHz |
| UART TX | PD5（pin2） | 115200 8N1 调试输出（printf） |
| UART RX | PD6（pin3） | 115200 8N1 调试输入（主循环回显） |

> 引脚号依据 ST 官方数据手册 STM8S103F2/F3 第 5.2 节 TSSOP20 pinout：PD4 = pin1、PD5 = pin2、PD6 = pin3、PA3 = pin10、PB4 = pin12、PC3 = pin13、PC4 = pin14、PC5 = pin15、PC6 = pin16、PC7 = pin17、PD3 = pin20（pin4 为 NRST，pin18 为 PD1/SWIM）。
>
> 电机四个绕组输出为普通推挽 GPIO，严禁直接驱动电机线圈，必须经 L9110S / ULN2003 驱动板。

## 功能说明

### 按键（PD3，非阻塞 FSM）

按键行为按运行模式区分，上电默认「连续转」，长按切换模式。去抖 20ms、释放确认 30ms、双击窗口 200ms、长按判定 1000ms。

- 单击
  - 连续转模式下：电机 启动 / 停止 切换。启动：PA3 亮、BEEP 2kHz/100ms、串口 `MOTOR START`；停止：PA3 灭、BEEP 1kHz/100ms、串口 `MOTOR STOP`
  - 点动模式下：走一步（按当前方向前进一个半步，相位通电 `on_ms` 后自动断电），BEEP 1kHz/30ms、串口 `JOG STEP`；单击不影响 PA3，PA3 保持点动模式闪烁节奏
- 双击：正转（CW）↔ 反转（CCW）切换。PB4 反转灯 CCW 亮 / CW 灭，BEEP 4kHz/80ms，串口 `DIR CCW` / `DIR CW`
- 长按：连续转 ↔ 点动 模式切换。切换时先立即停机并熄灭 PA3，避免连续运转状态串入点动模式；进入点动模式后 PA3 转为 250ms 半周期闪烁（主循环用 `g_tick` 计时），回到连续转则恢复为随电机亮灭的运行灯；BEEP 4kHz/120ms 进入点动、2kHz/120ms 回到连续转；串口 `MODE JOG` / `MODE CONTINUOUS`

### 电位器调速（PC4 = AIN2）

- 采样节奏：自适应指数退避，间隔由 50ms 起，每完成一次采样 + 调档后 ×2（50 → 100 → 200 → 400 → 800 → 1000ms，到 1000ms 封顶）；本次 raw 与上次参与调档的 raw 之差超过 `POT_MOVE_LSB`（默认 8 LSB，约满量程 0.8%）时判定为正在转动，间隔立即回落到 50ms 快速跟踪（`POT_POLL_MS_MIN = 50U`、`POT_POLL_MS_MAX = 1000U`）
- ADC 值 0~1023 指数（对数刻度）映射到步进周期 1000ms → 6ms，不再是线性映射：
  `POT_PERIOD_EXP_TABLE[i] = round(6 × (1000/6)^(i/32))`，i = 0..32（33 项 `uint16_t` 只读表，编译期固化在 Flash，无浮点、无 `pow()`）
  索引：`level = (uint32_t)raw × 32 / 1023`（0..32），再取 `idx = 32 − level` 查表
- ADC = 0（逆时针到头）→ 1000ms/步（最慢，约 1 步/秒）；ADC = 1023（顺时针到头）→ 6ms/步（最快，约 167 步/秒），方向与原线性版一致（CCW = 慢）
- 对数刻度要点：每档相邻周期比值恒为约 1.173 = (1000/6)^(1/32)，即旋钮转过相同角度时速度按相同百分比变化，全量程手感均匀；33 档让最低速从原来的 50ms/步 拓展到 1000ms/步（慢 20 倍），而最高速仍保持 6ms/步 不变
- 完整 33 档查表值（ms/步，索引 0→32 由快到慢）：
  `6, 7, 8, 10, 11, 13, 16, 18, 22, 25, 30, 35, 41, 48, 56, 66, 77, 91, 107, 125, 147, 172, 202, 237, 278, 327, 383, 450, 528, 619, 726, 852, 1000`
- 调速范围：周期钳制在 6~1000ms/步（`main.c` 的 `MOTOR_PERIOD_MIN = 6U`、`MOTOR_PERIOD_MAX = 1000U`，与查表两端对齐）
- 溢出与越界保护：索引计算全程 `uint32_t` 中间量（`raw × 32` 不落回 16 位）；`raw` 先钳到 1023；`idx` 钳在 0..32，越界读表不可能发生
- 只有档位发生变化时才更新电机并打印 `pot=<raw> spd=<period> ms/step`，避免刷屏
- ADC 驱动：单次转换 + 每读 8 次平均滤波 + EOC 超时保护。输入稳定延时（settle）只在**通道切换时**执行一次：`ADC_t` 新增 `last_ch` 字段（`adc_init` 置 `0xFF` = 尚未采样），`adc_read` 中仅当 `self->ch != self->last_ch` 才调用 `adc_settle_delay()` 并更新 `last_ch`。语义是"通道变了才需要重新给采样电容充电"，同一通道连续读不再白等 ≈187µs（全工程只有 PC4 一个通道，实际只在第一次读时延时）
- ADC 故障不伪装成 0V：EOC 超时时 `adc_convert_once` 返回哨兵，`adc_read` **立即上抛** `ADC_RAW_ERROR`（`adc.h`，`0xFFFFU`），不参与平均、不再折算成 `0x0000`。契约是 `adc.read()` 返回 0~1023 才是有效读数，超出该范围即为错误。调用点处理：①初始化时若取到哨兵，用 `POT_PERIOD_EXP_TABLE[32]`（最慢档 1000ms）作安全默认并打印 `adc err, default slowest`；②主循环采样若取到哨兵，跳过 `adc_to_period` / `setSpeed`，不更新 `last_pot_raw`、不置 `pot_have_ref`，`pot_poll_ms` 回落到 `POT_POLL_MS_MIN` 以便尽快重试，并按 `adc err cnt=<n>`（首次立即报，之后每满 50 次再报）告警，恢复后打印一条 `adc ok` 并清零计数
- ADC 采样时钟：`SPSEL = 6` → `fADC = fMASTER/12 = 16MHz/12 = 1.333MHz`（手册要求 fADC 在 1~4MHz）。SPSEL 分频表不是等比序列：0=/2、1=/3、2=/4、3=/6、4=/8、5=/10、6=/12、7=/18，没有 /14 这一档；旧的 `SPSEL = 7`（/18 = 0.889MHz）低于 1MHz 下限，已改为 6（实参在 `main.c` 的 `ADC_Create(ADC_CH_POT, 6)`）
- ADC 结果对齐：`ADC1_CR2` 的 ALIGN 位在 `adc_init` 中显式写 0 = 左对齐（不依赖芯片复位值，也不允许改成右对齐）。左对齐下 `DRH[7:0] = DATA[9:2]`、`DRL[7:6] = DATA[1:0]`，合并公式为 `(DRH << 2) | (DRL >> 6)`，且必须先读 DRH 再读 DRL（读 DRH 会锁存 DRL），因此代码分两条独立语句读取。满量程 0~1023（旧代码把左对齐数据当右对齐拼，`DRL & 0x03` 取到的是恒 0 的保留位，读数低 2 位永远为 0、精度退化成 8 位、满量程只到 1020）

### 电机驱动（1-2 相励磁半步）

- 8 拍相位表：CW 沿表正向换相，CCW 反向换相
- `setSpeed` 将周期钳制在 [1, 1000] ms/步（`motor.h` 的 `MOTOR_PERIOD_LIMIT = 1000U`，与 `main.c` 指数曲线的慢端 1000ms 对齐，不会把 1000ms 截断成更小值）。写 `period_ms` 这一句用 `__asm__("sim")` / `__asm__("rim")` 包成临界区：`period_ms` 是 16 位而内核是 8 位，一次赋值是两条字节写指令，而 TIM1 的 1ms 中断里 `motor_onTime()` 会读它算 `off_ms = period_ms − pw`，中断落在两条字节写之间会读到新旧拼接的半值、产生一拍错误周期。钳制等纯本地计算留在临界区之外，窗口只有一条语句宽
- 两相状态机（ON 通电段 / OFF 断电段）：`Motor_t` 用 `state`（`MOTOR_STATE_ON` / `MOTOR_STATE_OFF`）与 `state_ms`（当前阶段剩余毫秒）描述每一步。每步开始时输出目标相序并装载 ON 段倒计时 `pw`（单线圈拍 = `on_ms`，双线圈拍 = `(on_ms + 1) / 2`，因双相同时通电电流翻倍而缩短）；`pw` 毫秒到点后四路输出全部清零（L9110S 输入全 0，线圈断电），装载 OFF 段倒计时 `period_ms − pw`；OFF 到点后执行下一次换相。因此 `period_ms` 是整步周期（含通电段），不是停转时长——慢速下线圈只在通电窗口内有电流，省电流、电机不发烫
- `on_ms` 由 `Motor_Create(ports, pins, on_ms)` 传入（`motor.h` 中 `MOTOR_ON_MS = 4U` 为推荐经验值）；取值应 ≤ 最小 period。本项目调速下限 6ms/步，全档位 `period ≥ 6ms > on_ms = 4ms`，因此每个档位都保留断电窗口：最快档 4ms 通 / 2ms 断，双线圈拍 2ms 通 / 4ms 断；最慢档单线圈拍 4ms 通 / 996ms 断，双线圈拍 2ms 通 / 998ms 断。`off_ms` 最大 998 远低于 `uint16_t` 上限 65535，无截断。若 `on_ms ≥ 当前 period`，`motor.c` 的减法保护会把 OFF 段回退为 1ms，不会产生无符号下溢导致的超长停顿
- 停止时释放全部线圈（输出低电平），避免长时间通电发热
- 单步点动 `stepOnce()`：调用时不做任何端口操作，只把 `next_step`（待执行步数）饱和自增 1（上限 0xFF，连点不丢步、不回绕），`running` 保持 0；随后由 `motor.onTime()` 走与连续转完全相同的 ON/OFF 倒计时把这一步走完——一次 OFF 段到期只消费一个 `next_step`，队列空后停在 OFF 且线圈已断电（点一次走一步）。`start` / `stop` / `init` / 构造均会把 `state`、`state_ms`、`next_step` 归位到确定初值，不会多走一步，也不会残留上一次的通电段

### L9110S 驱动板接线线序

- MCU → L9110S 输入（按驱动板丝印）：PC3 → A-1A、PC5 → A-1B、PC6 → B-1A、PC7 → B-1B
- 代码编号对照（仅用于阅读源码，不作接线依据）：IN1 = PC3、IN2 = PC7、IN3 = PC5、IN4 = PC6
- 绕组配对：A 相 = PC3(A+) + PC5(A-)，B 相 = PC6(B-) + PC7(B+)；即同一绕组的两根线必须接到驱动板的同一路通道（A-1A/A-1B 为一组，B-1A/B-1B 为一组），不可把同一绕组的两线跨组连接
- L9110S → 步进电机：A 相两线接 A-OA / A-OB，B 相两线接 B-OA / B-OB
- 供电：驱动板 VM 接 5V，GND 与 MCU 共地；电机按额定电压接到驱动板输出
- 相位表位序说明（重要）：`motor.c` 头部注释的 8 拍表按 `bit3 = IN1、bit2 = IN2、bit1 = IN3、bit0 = IN4` 的位序书写，而 `motor_write_phase()` 实际按 `bit0 = IN1、bit1 = IN2、bit2 = IN3、bit3 = IN4` 逐位写引脚（`pattern & (1 << i) → pin[i]`）。两者互为镜像，故 `phase_table` 的实际通电相序是注释序列的逆序，`dir = 0` 时物理转向与注释标称的 CW 相反。这是已确认的镜像关系，不是接线错误
- 转向调整：若电机转向与预期相反，交换任一组绕组的两根线（A-OA/A-OB 或 B-OA/B-OB 二者之一）即可，无需改代码

### 状态指示与提示音

- 上电默认双灯熄灭：`led.init` 内部会把输出置高点亮，main 中 init 后显式 `led_run.off` + `led_dir.off` 使其默灭，仅在动作时点亮
- 上电自检：BEEP 单次 2kHz/100ms，串口输出 `STM8S103 STEPPER ready`
- 按键提示音使用 `beep.once(档位, 毫秒)`，第一参数传 1 / 2 / 4 档位（对应 1k / 2k / 4kHz），非频率数值

### UART 调试（PD5 TX / PD6 RX）

- 115200 8N1；发送走 128 字节非阻塞环形缓冲，由主循环 `uart.tx_service()` 排空（按键回调运行在 TIM1 中断内，仅入队不阻塞）
- RXNE 中断接收，主循环回显收到的字节
- 打印内容：上电 `STM8S103 STEPPER ready`；按键事件 `MOTOR START` / `MOTOR STOP`（连续转）、`JOG STEP`（点动一步）、`DIR CW` / `DIR CCW`、`MODE JOG` / `MODE CONTINUOUS`；调速 `pot=<raw> spd=<period> ms/step`；ADC 故障告警 `adc err cnt=<n>`（首次 + 每 50 次）、初始化故障 `adc err, default slowest`、恢复 `adc ok`

### 运行机制

- 时钟：HSI 16MHz（`CLK_CKDIVR = 0x00`）。主频只在 `main.c` 的 `clock_setup()` 里设一次，`clock_setup()` 在 `uart.init()` 之前调用；`uart_init()` 不再改 `CLK_CKDIVR`（原实现会重复设置主频并覆盖调用方的时钟配置）
- 外设时钟门控：按位使能，不做整字节覆盖写。`uart_init()` 用 `CLK_PCKENR1 |= CLK_PCKENR1_UART1EN`（bit2 = 0x04），`tim1_setup()` 用 `CLK_PCKENR1 |= 0x80`（bit7）；原来 `CLK_PCKENR1 = 0xFF` 会把 I2C1/SPI1/TIM2/TIM3 一起打开并覆盖其他模块已配置的门控位
- 时间片：TIM1 1ms 更新中断驱动 `key.onTime` / `motor.onTime` / `beep.update`（提示音自动关闭）
- 主循环：`wfi` 等待中断唤醒（注释明确禁用 `halt`，因 TIM1 无法唤醒 halt），唤醒后先排空 UART，再按自适应指数退避间隔（50 → 1000ms）采样电位器

## 编译与烧录

在 cmd 中执行（避免 PowerShell 重定向编码问题）：

```bat
cd /d D:\stm8s_project\project2
build.bat
rem 或 make -f Makefile

rem 烧录（stm8flash，ST-Link v2）
D:\sdcc\bin\stm8flash.exe -c stlinkv2 -p stm8s103f3 -w project2.ihx
```

- `build.bat`：SDCC 命令行编译全部源文件 → `project2.ihx`，编译选项必须包含 `--std-c99`（否则 UART printf 变参与声明会乱码或报错），生成后自动 packihx 并清理中间文件
- `Makefile`：`make -f Makefile` 可编译；提供 `all` / `hex` / `size` / `flash` / `flash_uart` / `clean` / `help` 等目标（`flash_uart` 目标依赖串口参数，详见 Makefile 内注释）

## 文件清单

| 文件 | 说明 |
|---|---|
| main.c | 应用编排 + 时钟/TIM1/EXTI/UART1_RX 中断向量表 |
| motor.h / motor.c | 步进电机对象：1-2 相半步换相、启停、单步点动（stepOnce）、正反转、调速钳制 |
| key.h / key.c | 按键对象：任意端口 EXTI 参数化注册，非阻塞单击/双击/长按 FSM |
| adc.h / adc.c | ADC1 对象：PC4(AIN2) 单次转换 + 平均滤波 + EOC 超时 |
| uart.h / uart.c | UART1 对象：115200 8N1，非阻塞 TX 环形缓冲 + RX 中断 |
| led.h / led.c | LED 对象：启停/反转指示灯（active_high 可配） |
| beep.h / beep.c | 硬件 BEEP 对象：PD4，1 / 2 / 4 kHz 定时发声（LSI 时钟） |
| build.bat | Windows 编译脚本（SDCC + `--std-c99`） |
| Makefile | Make 构建脚本（含烧录目标） |

## 注意事项

- 引脚约束：STM8S103F3P6 为 TSSOP20，仅引出部分 GPIO；本项目使用的 PA3 / PB4 / PC3 / PC4 / PC5 / PC6 / PC7 / PD3 / PD4 / PD5 / PD6 均为实际可用引脚（PB3 在 TSSOP20 上未引出，不可使用；PB4 / PB5 为真开漏，只能低电平驱动）。PA4~PA7 等未引出引脚不可配置为输出使用，扩展功能前务必对照数据手册确认
- 调速下限取 6ms/步的理由：更快的 1~2ms/步（500~1000 半步/秒）远超 5V 小步进电机的牵入频率，实测出现失步、堵转；且周期缩到接近或小于 `on_ms`（4ms）时断电窗口消失，失去省电效果。实测数据：最快档（6ms/步）线圈电流约 0.4A，原 50ms/步 档约 0.02A（1000ms/步 档仅在通电 4ms 内有电流，占空比 0.4%，平均电流更低），连续运行电机不发热。更换电机或驱动板时需调 `main.c` 中 `MOTOR_PERIOD_MIN` / `MOTOR_PERIOD_MAX` 宏与查表内容，并同步 `motor.h` 的 `MOTOR_ON_MS`；若把 `MOTOR_PERIOD_MIN` 调到 ≤ 4U，`on_ms ≥ period` 时 OFF 段由 `motor.c` 的减法保护回退为 1ms（不会出现无符号下溢导致的超长停顿），但会显著失去省电效果
- 驱动板必需：绕组信号为 GPIO 电平，不可直连电机线圈，必须接 L9110S / ULN2003 驱动板
- 编译选项：必须使用 `--std-c99`，否则 UART 的 `printf` 变参调用（`%u` 等）与代码声明会出问题；推荐直接运行 `build.bat`
- BEEP 档位参数：`beep.once(&beep, khz, ms)` 的 `khz` 传档位 1 / 2 / 4，不是任意频率值
- LED 上电状态：上电两灯默认熄灭（`main` 中 init 后显式 off），如需常亮请调整 `main.c` 初始化顺序
- UART 输出：发送为环形缓冲非阻塞设计，主循环必须持续调用 `uart.tx_service()` 排空，否则输出滞留
- 低功耗唤醒：主循环使用 `wfi` 而非 `halt`（TIM1 更新中断无法唤醒 halt）
- 电源与电平：电位器参考 3.3V（与 MCU 同电源域）；驱动板电源按电机额定电压独立供电

*（内容由AI生成，仅供参考）*
