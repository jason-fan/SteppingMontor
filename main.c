/**
 * STM8S103F3P6 2-Phase 4-Wire Stepper Motor Control (Object-Oriented)
 *
 * Target MCU:   STM8S103F3P6
 * Compiler:     SDCC (Small Device C Compiler)
 * Clock:        HSI 16MHz
 *
 * Peripherals / pins (TSSOP20):
 *   KEY       PD3  -> single button (active low, pull-up, TSSOP20 pin20)
 *                    click       : CONTINUOUS mode -> motor RUN / STOP toggle
 *                                  JOG mode        -> advance one single step
 *                    double-click: toggle CW / CCW rotation direction
 *                    long-press  : toggle CONTINUOUS / JOG operation mode
 *   MOTOR     PC3=IN1(A-1A) PC7=IN2(B-1B) PC5=IN3(A-1B) PC6=IN4(B-1A)
 *             (L9110s/ULN2003 driver board; names in brackets are the
 *              L9110S silkscreen labels, A phase = PC3+PC5 (IN1+IN3),
 *              B phase = PC6+PC7 (IN4+IN2))
 *   POT       PC4  -> AIN2, 10k potentiometer wiper -> motor speed
 *   LED_RUN   PA3  -> mode/run indicator (active high):
 *                    CONTINUOUS: lit while stepping, dark when stopped
 *                    JOG       : blinks every JOG_BLINK_MS (mode marker)
 *   LED_DIR   PB4  -> reverse indicator (lit when CCW, ACTIVE LOW:
 *                    PB4 (pin12) is a TRUE OPEN-DRAIN pad, it cannot
 *                    drive high -> LED anode to VDD, cathode to PB4)
 *   BEEP      PD4  -> hardware BEEP (key tone)
 *   UART      PD5 TX / PD6 RX, 115200 8N1 debug output
 *
 * Event loop: TIM1 hard interrupt schedules key.onTime / motor.onTime;
 * the main loop drains the UART TX ring buffer and samples the
 * potentiometer with an adaptive (exponential backoff) interval:
 * 50ms while the pot is being turned, doubling up to 1000ms when idle.
 *
 * Motor timing: the driver runs a two-phase state machine (motor.c).
 * Each step period is period_ms long and is split into an ON phase
 * (coils energized for pw ms) followed by an OFF phase (all four
 * outputs 0 for period_ms - pw ms) — period_ms is the WHOLE step
 * period, coil-on window included, not an idle duration. pw is 4 ms
 * for single-coil phases and 2 ms for double-coil phases with
 *     Motor_Create(MOTOR_PORTS, MOTOR_PINS, MOTOR_ON_MS);
 * so keep on_ms <= the smallest period (6 ms) to guarantee an OFF
 * window at every speed.
 *
 * Wiring note: the four coil outputs are ordinary push-pull GPIO pins.
 * For a 2-phase 4-wire motor use a L9110s/ULN2003 driver board
 * so the MCU never drives the coils directly.
 */

#include <stdint.h>
#include "motor.h"
#include "key.h"
#include "adc.h"
#include "uart.h"
#include "led.h"
#include "beep.h"

/* ========== User Configuration ========== */

#define GPIO_PA         0x5000
#define GPIO_PB         0x5005
#define GPIO_PC         0x500A
#define GPIO_PD         0x500F

/* Single button: PD3 (active low, internal pull-up, EXTI3) */
#define KEY_PORT        GPIO_PD
#define KEY_PIN         3

/* Stepper motor phase outputs (IN1..IN4)
 * MOTOR_PINS[i] is the pin of IN(i+1):
 *   IN1 = PC3 (A+ / L9110S A-1A), IN2 = PC7 (B+ / L9110S B-1B),
 *   IN3 = PC5 (A- / L9110S A-1B), IN4 = PC6 (B- / L9110S B-1A)
 * Winding A = IN1+IN3 = PC3+PC5, winding B = IN2+IN4 = PC7+PC6.
 * Wire by the L9110S silkscreen (A-1A/A-1B one channel, B-1A/B-1B the
 * other); the IN numbers are code references only, not wiring labels. */
static const uint16_t MOTOR_PORTS[4] = { GPIO_PC, GPIO_PC, GPIO_PC, GPIO_PC };
static const uint8_t  MOTOR_PINS[4]  = { 3,       7,       5,       6       };

/* Potentiometer ADC (PC4 = AIN2) */
#define ADC_CH_POT      ADC_CH_PC4

/* Run indicator LED (PA3, active high — adjust to your board) */
#define LED_RUN_PORT    GPIO_PA
#define LED_RUN_PIN     3

/* Reverse indicator LED (PB4 = TSSOP20 pin12, I2C_SCL, unused here;
 * TRUE OPEN-DRAIN pad -> ACTIVE LOW; PB3 does NOT exist on TSSOP20) */
#define LED_DIR_PORT    GPIO_PB
#define LED_DIR_PIN     4

/* ========== STM8S103 Register Definitions ========== */

#define CLK_CKDIVR      (*(volatile uint8_t *)0x50C6)

/* ========== Clock Configuration ========== */

static void clock_setup(void)
{
    CLK_CKDIVR = 0x00;  /* HSI 16MHz, no division */
}

/* ========== Global Objects (file-level, accessible from ISR) ========== */

static Motor_t motor;                  /* stepper motor object        */
static KEY_t   key;                    /* PD3: single button          */
static ADC_t   adc;                    /* ADC1 object (PC4 pot)       */
static UART_t  uart;                   /* UART1 debug link            */
static LED_t   led_run;                /* PA3: run indicator          */
static LED_t   led_dir;                /* PB3: reverse indicator      */
static Beep_t  beep;                   /* BEEP object (PD4)           */

/* Global 1ms tick, incremented by TIM1 ISR */
volatile uint16_t g_tick = 0;

/* ========== Operation Mode (toggled by long-press) ==========
 * MODE_CONTINUOUS: click = start/stop continuous stepping
 * MODE_JOG       : click = advance exactly one step               */
#define MODE_CONTINUOUS   0U
#define MODE_JOG          1U

/* Written by the key callbacks (TIM1 ISR context), read by the main loop
 * (mode check, PA3 blink). MUST be volatile: without it the compiler may
 * hoist the read out of while(1) and the mode switch would never be seen. */
static volatile uint8_t jog_mode = MODE_CONTINUOUS;

/* JOG mode indication: PA3 blinks with a JOG_BLINK_MS half period so the
 * current mode is visible at a glance, and it is NOT touched by clicks
 * (a jog click only steps the motor). The blink is toggled from the main
 * loop off g_tick — the key callbacks run inside the TIM1 ISR and must
 * never block or busy-wait. In CONTINUOUS mode PA3 is a plain run lamp:
 * lit while stepping, dark when stopped. */
#define JOG_BLINK_MS      250U
static volatile uint8_t  jog_blink_on   = 0;
static volatile uint16_t jog_blink_tick = 0;

/* ========== Speed Mapping (potentiometer -> ms per step, EXPONENTIAL) ==========
 * ADC 0..1023 no longer maps LINEARLY onto 50..6 ms. It now walks an
 * exponential (logarithmic-scale) curve from 1000 ms/step down to 6 ms/step,
 * so the knob feels even across its whole travel: every equal turn multiplies
 * the step rate by the same factor instead of adding the same number of ms.
 *   pot full CCW (raw = 0)    -> 1000 ms/step (slowest, 1 step/s)
 *   pot full CW  (raw = 1023) ->    6 ms/step (fastest, ~167 steps/s)
 * Direction is unchanged from the old linear version (CCW = slow).
 * The curve is a 33-entry uint16_t table living in flash (const, no RAM):
 *   POT_PERIOD_EXP_TABLE[i] = round(6 * (1000/6)^(i/32)),  i = 0..32
 *   [0]  = 6 ms (fastest) ... [32] = 1000 ms (slowest)
 *   strictly increasing, adjacent ratio ~1.173 = (1000/6)^(1/32) (13 radix
 *   steps per octave-ish 2.37x), i.e. an equal-percentage speed law.
 * No floating point and no pow(): the table is folded at compile time, so the
 * per-sample cost is one uint32_t multiply, one divide and one table read.
 * period_ms is the WHOLE step period, coil-on window (pw) included: the driver
 * splits it into a pw ms ON phase plus a (period_ms - pw) ms OFF phase, so the
 * commutation cadence is one phase change per period_ms. At the fastest step
 * (6 ms) on_ms = 4 ms still leaves a 2 ms off-window — no continuous
 * energization at any pot position; at the slowest step (1000 ms) the off
 * window is ~996 ms (998 ms on the double-coil phases, pw = 2 ms). Values
 * are clamped by motor.setSpeed() anyway. */
#define MOTOR_PERIOD_MAX   1000U   /* slowest step; must equal table[32] */
#define MOTOR_PERIOD_MIN   6U      /* fastest step; must equal table[0]  */

/* ========== Adaptive Pot Sampling (exponential backoff) ==========
 * 电位器采样不再用固定间隔，改为"动则快采、静则慢采"的指数退避：
 *   POT_POLL_MS_MIN : 起始 / 最快轮询间隔（ms），也是"电位器被转动"后的回落值
 *   POT_POLL_MS_MAX : 轮询间隔上限（ms），到达后不再增长（饱和封顶）
 *   POT_MOVE_LSB    : 判定"电位器被转动"的原始 ADC 差值阈值（LSB）
 * 每完成一次采样并完成调档后，间隔 ×2 递增，序列：
 *   50 -> 100 -> 200 -> 400 -> 800 -> 1000(封顶，不再增长)
 * 若本次 raw 与上次参与调档的 raw 之差 > POT_MOVE_LSB（默认 8 LSB，
 * 约满量程 0.8%，可滤掉 ADC 噪声），判为正在转动 -> 间隔立即重置回
 * POT_POLL_MS_MIN，换档紧跟手感；电位器静止时采样越来越慢，省掉无谓转换。
 * 实现约束：计时只用 1ms 计数 g_tick 的无符号差值（uint16 自然回绕，
 * 65536ms 一个周期，差值比较 <= 1000 不受回绕影响）；递增只用左移
 * （×2），无浮点、无 32 位乘法、无阻塞延时。 */
#define POT_POLL_MS_MIN   50U
#define POT_POLL_MS_MAX   1000U
#define POT_MOVE_LSB      8U

/* Exponential speed table: round(6 * (1000/6)^(i/32)), i = 0..32.
 * [0] = 6 ms (fastest, pot fully CW) ... [32] = 1000 ms (slowest, fully CCW).
 * Strictly increasing; adjacent ratio 1.10..1.25 around the ideal 1.173
 * (the spread is the rounding quantisation at the low end, 6->7->8 ms). */
static const uint16_t POT_PERIOD_EXP_TABLE[33] = {
       6U,   7U,   8U,  10U,  11U,  13U,  16U,  18U,
      22U,  25U,  30U,  35U,  41U,  48U,  56U,  66U,
      77U,  91U, 107U, 125U, 147U, 172U, 202U, 237U,
     278U, 327U, 383U, 450U, 528U, 619U, 726U, 852U,
    1000U
};

/* Compile-time guard: the table must hold exactly 33 entries (0..32).
 * Negative array size = compile error, no runtime cost, no RAM. */
typedef char pot_period_exp_table_must_have_33_entries
    [(sizeof(POT_PERIOD_EXP_TABLE) / sizeof(POT_PERIOD_EXP_TABLE[0]) == 33) ? 1 : -1];

static uint16_t adc_to_period(uint16_t adc_raw)
{
    /* level = 0 (raw = 0, pot CCW) ... 32 (raw = 1023, pot CW).
     * (uint32_t) cast keeps raw * 32 inside 32 bits: raw <= 1023 so the
     * product is <= 32736 and would fit 16 bits today, but the cast keeps
     * the expression correct if the ADC width or the radix ever grows. */
    uint16_t level;
    uint16_t idx;

    if (adc_raw > 1023U) {
        adc_raw = 1023U;                  /* 10-bit ADC range guard        */
    }

    level = (uint16_t)(((uint32_t)adc_raw * 32UL) / 1023UL);   /* 0..32 */

    /* Mirror the index: raw = 0 -> idx 32 (1000 ms, slowest),
     *                  raw = 1023 -> idx 0 (6 ms, fastest).
     * 32 - level can never go negative (level <= 32) and never exceeds 32,
     * so the table read below is bounds-safe; the explicit clamp is a
     * belt-and-braces guard against a future change of the level formula. */
    idx = (uint16_t)(32U - level);
    if (idx > 32U) {
        idx = 32U;
    }

    return POT_PERIOD_EXP_TABLE[idx];
}

/* ========== Key Event Callbacks ==========
 * UART output enqueues into the TX ring buffer only, so these
 * callbacks are safe to run inside the TIM1 ISR; the main loop
 * drains the buffer with uart.tx_service(). */

/* Click:
 *   CONTINUOUS mode -> toggle motor RUN / STOP (PA3 follows the motor)
 *   JOG mode        -> queue exactly one step in next_step; motor.onTime()
 *                      consumes it at the next OFF expiry and drives the
 *                      ON/OFF countdowns, so the coil-on time equals that
 *                      of a running step and the machine parks in OFF with
 *                      the coils released after the step.
 *                      PA3 keeps its mode blink pattern and is deliberately
 *                      left untouched here                                   */
static void key_onclick(KEY_t *self)
{
    (void)self;

    if (jog_mode == MODE_JOG) {
        motor.stepOnce(&motor);        /* one step in current direction */
        beep.once(&beep, 1, 30);       /* jog tone: 1kHz / 30ms */
        uart.puts(&uart, "JOG STEP\r\n");
        return;
    }

    if (motor.isRunning(&motor)) {
        motor.stop(&motor);
        led_run.off(&led_run);
        beep.once(&beep, 1, 100);      /* stop  tone: 1kHz / 100ms */
        uart.puts(&uart, "MOTOR STOP\r\n");
    } else {
        motor.start(&motor);
        led_run.on(&led_run);
        beep.once(&beep, 2, 100);      /* start tone: 2kHz / 100ms */
        uart.puts(&uart, "MOTOR START\r\n");
    }
}

/* Double-click: toggle rotation direction CW <-> CCW, update reverse LED */
static void key_ondbclick(KEY_t *self)
{
    (void)self;
    motor.toggleDir(&motor);
    if (motor.dir) {
        led_dir.on(&led_dir);          /* CCW: reverse LED on  */
    } else {
        led_dir.off(&led_dir);         /* CW:  reverse LED off */
    }
    beep.once(&beep, 4, 80);        /* direction tone: 4kHz / 80ms */
    uart.printf(&uart, "DIR %s\r\n",
                motor.dir ? "CCW" : "CW");
}


/* Long-press: toggle CONTINUOUS <-> JOG operation mode.
 * The motor is stopped immediately on any mode change (coils released,
 * PA3 dark), so a running continuous motion never leaks into jog mode.
 * In JOG mode PA3 switches to a blink pattern as the mode indicator. */
static void key_onlongpress(KEY_t *self)
{
    (void)self;

    if (motor.isRunning(&motor)) {
        motor.stop(&motor);
    }
    led_run.off(&led_run);         /* both modes start from PA3 dark */

    jog_mode = (uint8_t)(jog_mode ^ 1U);

    if (jog_mode == MODE_JOG) {
        /* Arm the mode blink: lit immediately, then toggled by main loop */
        jog_blink_on   = 1;
        jog_blink_tick = g_tick;
        led_run.on(&led_run);
        beep.once(&beep, 4, 120);      /* mode tone: 4kHz / 120ms */
        uart.puts(&uart, "MODE JOG\r\n");
    } else {
        jog_blink_on = 0;              /* back to a plain run lamp */
        beep.once(&beep, 2, 120);      /* mode tone: 2kHz / 120ms */
        uart.puts(&uart, "MODE CONTINUOUS\r\n");
    }
}

/* ========== TIM1 1ms Time-Slice Timer (STM8S103) ========== */

#define TIM1_CR1    (*(volatile uint8_t *)0x5250)
#define TIM1_IER    (*(volatile uint8_t *)0x5254)
#define TIM1_SR1    (*(volatile uint8_t *)0x5255)
#define TIM1_EGR    (*(volatile uint8_t *)0x5257)
#define TIM1_PSCRH  (*(volatile uint8_t *)0x5260)
#define TIM1_PSCRL  (*(volatile uint8_t *)0x5261)
#define TIM1_ARRH   (*(volatile uint8_t *)0x5262)
#define TIM1_ARRL   (*(volatile uint8_t *)0x5263)

/* HSI 16MHz, prescaler /16 -> 1MHz counter, ARR=1000 -> 1ms update */
static void tim1_setup(void)
{
    CLK_PCKENR1 |= 0x80;   /* bit7 = TIM1 clock gate, explicit enable */
    TIM1_PSCRH = 0x00;   /* prescaler high byte (16-bit prescaler = 16) */
    TIM1_PSCRL = 15;     /* /16 */
    TIM1_ARRH  = 0x03;   /* auto-reload 999 (0x03E7) = exactly 1 ms */
    TIM1_ARRL  = 0xE7;
    TIM1_EGR  |= 0x01;   /* UG: generate update to load prescaler/ARR */
    TIM1_IER  |= 0x01;   /* UIE: enable update interrupt */
    TIM1_CR1  |= 0x01;   /* CEN: start counter */
}

/* ========== TIM1 Update ISR ==========
 * STM8S103F3 TIM1_UPD_OVF_TRG_BRK vector = 0x8034 (IRQ 11)
 * -> SDCC __interrupt(11). */

void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(11)
{
    TIM1_SR1 &= (uint8_t)~0x01;      /* clear UIF (must do before exit) */

    g_tick++;                        /* advance global 1ms tick */

    /* Time-slice scheduling: non-blocking periodic events only. */
    key.onTime(&key, g_tick);
    motor.onTime(&motor, g_tick);
    beep.update(&beep);             /* auto-off timed beeps (was missing!) */
}

/* ========== EXTI0..EXTI3 ISRs (per-port) ==========
 * 硬件向量：EXTI0(Port A)=0x8014(IRQ3), EXTI1(Port B)=0x8018(IRQ4),
 *           EXTI2(Port C)=0x801C(IRQ5), EXTI3(Port D)=0x8020(IRQ6).
 * SDCC 映射 __interrupt(n) -> 0x8000+(n+2)*4，故分别用 (3)(4)(5)(6)。
 * 单键在 Port D，共享 EXTI3；key_exti_service(3) 一次扫描即可。
 * 其余组保留以支持 key.h 任意端口按键注册。
 */
void EXTI0_IRQHandler(void) __interrupt(3) { key_exti_service(0); }
void EXTI1_IRQHandler(void) __interrupt(4) { key_exti_service(1); }
void EXTI2_IRQHandler(void) __interrupt(5) { key_exti_service(2); }
void EXTI3_IRQHandler(void) __interrupt(6) { key_exti_service(3); }

/* ========== UART1 RX ISR Stub (must live in the main unit!) ==========
 * SDCC 4.5.0 stm8 后端只在"含 main 的编译单元"生成 __interrupt_vect 向量表。
 * UART1_RX vector = 0x8050 (IRQ 18) -> __interrupt(18)。
 * 向量壳放在本文件，实际处理委托给 uart 模块的 uart_rx_irq_handler()。
 */
void UART1_RX_IRQHandler(void) __interrupt(18)
{
    uart_rx_irq_handler();
}

/* ========== Main Entry ========== */

int main(void)
{
    uint16_t last_pot_tick = 0;      /* 上次采样时刻（g_tick, 1ms 计数）      */
    uint16_t pot_poll_ms   = POT_POLL_MS_MIN; /* 当前轮询间隔（指数递增, ms）   */
    uint16_t last_pot_raw  = 0;      /* 上次参与调档的 ADC 原始值             */
    uint8_t  pot_have_ref  = 0;      /* 0 = 尚未建立基准，首轮必采            */
    uint16_t last_period   = 0xFFFF; /* force first speed report              */
    uint16_t adc_err_cnt   = 0;      /* 连续 ADC 采样失败次数（0 = 正常）      */

    /* Create objects (bind methods + defaults) */
    motor   = Motor_Create(MOTOR_PORTS, MOTOR_PINS, MOTOR_ON_MS); /* 4ms 通电时间窗 */
    key     = KEY_Create(KEY_PORT, KEY_PIN);
    uart    = UART_Create(UART1_BASE);
    /* SPSEL = 110 -> fADC = fMASTER/12 = 16MHz/12 = 1.333MHz
     * (datasheet fADC range is 1MHz..4MHz; SPSEL=111 would give /18 =
     * 0.889MHz, i.e. BELOW the 1MHz minimum. Note SPSEL is not a power-of-
     * two ladder: 0=/2, 1=/3, 2=/4, 3=/6, 4=/8, 5=/10, 6=/12, 7=/18.)
     * 1.333MHz = 75ns conversion clock; every adc.read() averages 8 samples. */
    adc     = ADC_Create(ADC_CH_POT, 6);   /* PC4 电位器, SPSEL=110 -> fADC=fMASTER/12 */
    led_run = LED_Create(LED_RUN_PORT, LED_RUN_PIN, 1); /* active high */
    led_dir = LED_Create(LED_DIR_PORT, LED_DIR_PIN, 0); /* active LOW: PB4 open-drain */
    beep    = BEEP_Create();                /* 硬件 BEEP: PD4 */

    clock_setup();                  /* HSI 16MHz */

    /* Init peripherals */
    motor.init(&motor);             /* PC3/PC7/PC5/PC6 push-pull, coils off */
    key.init(&key);                 /* PD3 input + pull-up + EXTI3 */
    uart.init(&uart);               /* UART1 115200 8N1, TX PD5 / RX PD6 */
    adc.init(&adc);                 /* ADC1 single conversion, AIN2 = PC4 */
    led_run.init(&led_run);         /* PA3 push-pull output */
    led_dir.init(&led_dir);         /* PB4 open-drain output (low = lit) */
    beep.init(&beep);               /* LSI on + PD4 BEEP push-pull (was missing!) */

    /* Bind key callbacks: click       = RUN/STOP (continuous) or one step (jog),
     *                     double-click = direction toggle CW <-> CCW,
     *                     long-press   = CONTINUOUS <-> JOG mode toggle */
    key.onclick     = &key_onclick;
    key.ondblclick = &key_ondbclick;
    key.onlongpress = &key_onlongpress;

    /* Power-on default: both LEDs OFF (lit only on actual action).
     * led.init drives the output high (lit); explicitly clear both
     * now so the run/reverse indicators are dark at startup. */
    led_run.off(&led_run);
    led_dir.off(&led_dir);

    /* Default speed from initial pot position.
     * adc.read() can return ADC_RAW_ERROR (EOC poll timeout: dead wiper,
     * open wiring, ADC clock gate closed). The sentinel is outside 0..1023,
     * so feeding it to adc_to_period() would index the table with garbage.
     * On a fault, start from POT_PERIOD_EXP_TABLE[32] - the slowest step
     * (1000 ms) - as the safe default instead of pretending the pot sits at
     * 0 V, and report it once. uart.printf() only enqueues into the TX ring
     * buffer (drained later by uart.tx_service()), so it neither blocks nor
     * needs interrupts. */
    {
        uint16_t raw0 = adc.read(&adc);

        if (raw0 == ADC_RAW_ERROR) {
            motor.setSpeed(&motor, POT_PERIOD_EXP_TABLE[32]);   /* 1000 ms */
            uart.printf(&uart, "adc err, default slowest\r\n");
        } else {
            motor.setSpeed(&motor, adc_to_period(raw0));
        }
    }

    /* Re-close the interrupt gate: motor.setSpeed() wraps its period_ms store
     * in a sim/rim critical section, so it returns with interrupts ENABLED.
     * tim1_setup() (below) arms TIM1_IER.UIE, and the beep.once() store that
     * follows must still be atomic (see its comment) - so disable interrupts
     * again here and let the explicit "rim" below do the single official
     * enable. */
    __asm__("sim");

    tim1_setup();                   /* TIM1: 1ms update interrupt */

    /* Power-on self test: single 2kHz beep for 100ms on PD4.
     * Non-blocking: beep.once() only arms the counter; the tone is
     * actually produced by beep.update() inside the TIM1 1ms ISR.
     * Order matters: arm the counter with interrupts still DISABLED, then
     * enable them. beep.once() stores a 16-bit `remaining` field; if a TIM1
     * tick could fire in the middle of that store (which it would after
     * "rim"), the ISR would decrement a half-written value and could leave
     * the tone on for a garbage duration. */
    beep.once(&beep, 2, 100);

    __asm__("rim");                 /* enable global interrupts */

    uart.puts(&uart, "STM8S103 STEPPER ready\r\n");

    /* Event-driven main loop with buffered UART TX and adaptive pot
     * sampling (指数退避：电位器转动时 50ms，静止时 ×2 递增至 1000ms 封顶).
     * NOTE: 不能用 halt（TIM1 无法唤醒 halt）；用 wfi：CPU 暂停但外设时钟
     * 继续，任何中断（含 TIM1 1ms）唤醒后自动进入 ISR。 */
    while (1) {
        /* Drain pending TX bytes */
        uart.tx_service(&uart);

        /* JOG mode indicator: toggle PA3 every JOG_BLINK_MS.
         * CONTINUOUS mode leaves PA3 entirely under the key handlers
         * (lit while running, dark when stopped). */
        if (jog_mode == MODE_JOG
            && (uint16_t)(g_tick - jog_blink_tick) >= JOG_BLINK_MS) {
            jog_blink_tick = g_tick;
            jog_blink_on   = (uint8_t)(jog_blink_on ^ 1U);
            if (jog_blink_on) {
                led_run.on(&led_run);
            } else {
                led_run.off(&led_run);
            }
        }

        /* UART RX echo */
        if (uart.rx_flag) {
            uart.rx_flag = 0;
            uart.putc(&uart, (char)uart.rx_byte);
        }

        /* ADC: PC4 电位器自适应采样（指数退避），替代原来的固定 200ms。
         *   间隔 pot_poll_ms 由 POT_POLL_MS_MIN(50ms) 起，每完成一次采样+调档
         *   后 ×2：50 / 100 / 200 / 400 / 800 / 1000，到 POT_POLL_MS_MAX 封顶。
         *   若本次 raw 与上次参与调档的 raw 之差超过 POT_MOVE_LSB，说明电位器
         *   正被转动 -> 间隔立刻回落 POT_POLL_MS_MIN，下一轮马上复核新档位。
         *   计时用 g_tick 无符号差值（抗 65536ms 回绕）；递增只用左移，纯整数。
         *   仅当档位变化时才更新电机并打印（保持原有输出格式不变）。 */
        if (pot_have_ref == 0U
            || (uint16_t)(g_tick - last_pot_tick) >= pot_poll_ms) {
            uint16_t raw    = adc.read(&adc);
            uint16_t delta;

            last_pot_tick = g_tick;

            if (raw == ADC_RAW_ERROR) {
                /* ADC fault (EOC timeout). Never map the sentinel to a speed:
                 * keep the motor at its last good period, do NOT touch
                 * last_pot_raw / pot_have_ref (the next good sample must be
                 * compared against the last REAL position, and a fault on the
                 * very first sample must not fake a reference at 0), and retry
                 * at the fastest interval instead of backing off - a fault has
                 * to be re-checked as soon as the hardware recovers.
                 * Reporting: warn on the first failure, then only every 50th,
                 * and reset the counter on recovery so the next fault is
                 * announced again. uart.printf() only fills the TX ring
                 * buffer (drained by uart.tx_service() in this loop), so it
                 * cannot block and is safe here. */
                pot_poll_ms = POT_POLL_MS_MIN;
                adc_err_cnt++;
                if (adc_err_cnt == 1U
                    || (uint16_t)(adc_err_cnt % 50U) == 0U) {
                    uart.printf(&uart, "adc err cnt=%u\r\n",
                                (unsigned int)adc_err_cnt);
                }
            } else {
                uint16_t period = adc_to_period(raw);

                if (adc_err_cnt != 0U) {
                    adc_err_cnt = 0U;
                    uart.printf(&uart, "adc ok\r\n");
                }

                /* |raw - last_pot_raw|：分支相减，饱和度型写法，不会无符号下溢 */
                if (pot_have_ref == 0U) {
                    delta = 0U;                     /* 首轮建立基准，不算"转动"  */
                } else if (raw >= last_pot_raw) {
                    delta = (uint16_t)(raw - last_pot_raw);
                } else {
                    delta = (uint16_t)(last_pot_raw - raw);
                }

                if (period != last_period) {
                    last_period = period;
                    motor.setSpeed(&motor, period);
                    uart.printf(&uart, "pot=%u spd=%u ms/step\r\n",
                                (unsigned int)raw, (unsigned int)period);
                }
                last_pot_raw = raw;
                pot_have_ref = 1U;

                /* 指数退避：转动 -> 回落最快；静止 -> ×2 直到封顶 */
                if (delta > POT_MOVE_LSB) {
                    pot_poll_ms = POT_POLL_MS_MIN;
                } else if (pot_poll_ms < POT_POLL_MS_MAX) {
                    uint16_t next = (uint16_t)(pot_poll_ms << 1); /* ×2 = 左移 1 位 */
                    pot_poll_ms = (next > POT_POLL_MS_MAX) ? POT_POLL_MS_MAX : next;
                }
            }
        }

        __asm__("wfi");             /* wait-for-interrupt */
    }

    // return 0;
}
