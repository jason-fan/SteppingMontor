/**
 * STM8S103F3P6 2-Phase 4-Wire Stepper Motor Control (Object-Oriented)
 *
 * Target MCU:   STM8S103F3P6
 * Compiler:     SDCC (Small Device C Compiler)
 * Clock:        HSI 16MHz
 *
 * Peripherals / pins (TSSOP20):
 *   KEY       PD3  -> single button (active low, pull-up, TSSOP20 pin2)
 *                    click    : toggle motor RUN / STOP
 *                    long-press: toggle CW / CCW
 *   MOTOR     PC3=IN1 PC7=IN2 PC5=IN3 PC6=IN4  (L9110s/ULN2003 driver board)
 *   POT       PC4  -> AIN2, 10k potentiometer wiper -> motor speed
 *   LED_RUN   PA3  -> running indicator (active high)
 *   LED_DIR   PB3  -> reverse indicator (lit when CCW, active high)
 *   BEEP      PD4  -> hardware BEEP (key tone)
 *   UART      PD5 TX / PD6 RX, 115200 8N1 debug output
 *
 * Event loop: TIM1 hard interrupt schedules key.onTime / motor.onTime;
 * the main loop drains the UART TX ring buffer and samples the
 * potentiometer every 200ms to update motor speed.
 *
 * Motor power window: every step energizes the coils for on_ms and
 * releases them (all four outputs 0) until the next step — coil
 * current flows only inside the window. on_ms is passed to
 * Motor_Create (4 ms empirical default, MOTOR_ON_MS in motor.h);
 * keep it <= smallest period, or the motor runs always-on.
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

/* Stepper motor phase outputs (IN1..IN4) */
static const uint16_t MOTOR_PORTS[4] = { GPIO_PC, GPIO_PC, GPIO_PC, GPIO_PC };
static const uint8_t  MOTOR_PINS[4]  = { 3,       7,       5,       6       };

/* Potentiometer ADC (PC4 = AIN2) */
#define ADC_CH_POT      ADC_CH_PC4

/* Run indicator LED (PA3, active high — adjust to your board) */
#define LED_RUN_PORT    GPIO_PA
#define LED_RUN_PIN     3

/* Reverse indicator LED (PB3 — PB0..PB3 are free on this design;
 * PB4/PB5 are freed (motor moved to PC3/PC7), active high) */
#define LED_DIR_PORT    GPIO_PB
#define LED_DIR_PIN     3

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

/* ========== Speed Mapping (potentiometer -> ms per step) ==========
 * ADC 0..1023 maps linearly to MOTOR_PERIOD_MAX..MOTOR_PERIOD_MIN:
 *   pot full CCW (0)   -> 50 ms/step (slowest)
 *   pot full CW (1023) -> 2  ms/step (fastest)
 * MIN=2U guards against 1ms/step stalling (L9110s/ULN2003 small-motor limit).
 * Values are clamped by motor.setSpeed anyway. */
#define MOTOR_PERIOD_MAX   50U
#define MOTOR_PERIOD_MIN   2U

static uint16_t adc_to_period(uint16_t adc_raw)
{
    /* period = MAX - raw*(MAX-MIN)/1023 */
    uint32_t span = (uint32_t)(MOTOR_PERIOD_MAX - MOTOR_PERIOD_MIN);
    uint16_t p = (uint16_t)((uint32_t)MOTOR_PERIOD_MAX
                 - (uint32_t)(((uint32_t)adc_raw * span) / 1023U));
    return p;
}

/* ========== Key Event Callbacks ==========
 * UART output enqueues into the TX ring buffer only, so these
 * callbacks are safe to run inside the TIM1 ISR; the main loop
 * drains the buffer with uart.tx_service(). */

/* Click: toggle motor RUN / STOP */
static void key_onclick(KEY_t *self)
{
    (void)self;
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

/* Click: toggle motor STOP */
static void key_ondbclick(KEY_t *self)
{
    (void)self;
    if (motor.isRunning(&motor)) {
        motor.stop(&motor);
        led_run.off(&led_run);
        uart.puts(&uart, "MOTOR STOP\r\n");
    }
    beep.once(&beep, 1, 100);      /* stop  tone: 1kHz / 100ms */
}


/* Long-press: toggle rotation direction, update reverse LED */
static void key_onlongpress(KEY_t *self)
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
    TIM1_ARRH  = 0x03;   /* auto-reload 1000 (0x03E8) */
    TIM1_ARRL  = 0xE8;
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
    uint16_t last_pot_tick = 0;      /* ADC 200ms sampling scheduling */
    uint16_t last_period   = 0xFFFF; /* force first speed report       */

    /* Create objects (bind methods + defaults) */
    motor   = Motor_Create(MOTOR_PORTS, MOTOR_PINS, MOTOR_ON_MS); /* 3ms 通电时间窗 */
    key     = KEY_Create(KEY_PORT, KEY_PIN);
    uart    = UART_Create(UART1_BASE);
    adc     = ADC_Create(ADC_CH_POT, 7);   /* PC4 电位器, SPSEL=/9, 长采样+平均滤波 */
    led_run = LED_Create(LED_RUN_PORT, LED_RUN_PIN, 1); /* active high */
    led_dir = LED_Create(LED_DIR_PORT, LED_DIR_PIN, 1); /* active high */
    beep    = BEEP_Create();                /* 硬件 BEEP: PD4 */

    clock_setup();                  /* HSI 16MHz */

    /* Init peripherals */
    motor.init(&motor);             /* PC3/PC7/PC5/PC6 push-pull, coils off */
    key.init(&key);                 /* PD3 input + pull-up + EXTI3 */
    uart.init(&uart);               /* UART1 115200 8N1, TX PD5 / RX PD6 */
    adc.init(&adc);                 /* ADC1 single conversion, AIN2 = PC4 */
    led_run.init(&led_run);         /* PA3 push-pull output */
    led_dir.init(&led_dir);         /* PB3 push-pull output */
    beep.init(&beep);               /* LSI on + PD4 BEEP push-pull (was missing!) */

    /* Bind key callbacks: click = start/stop toggle,
     *                    long-press = direction toggle */
    key.onclick     = &key_onclick;
    key.ondblclick = &key_ondbclick;
    key.onlongpress = &key_onlongpress;

    /* Power-on default: both LEDs OFF (lit only on actual action).
     * led.init drives the output high (lit); explicitly clear both
     * now so the run/reverse indicators are dark at startup. */
    led_run.off(&led_run);
    led_dir.off(&led_dir);

    /* Default speed from initial pot position */
    motor.setSpeed(&motor, adc_to_period(adc.read(&adc)));

    tim1_setup();                   /* TIM1: 1ms update interrupt */
    __asm__("rim");                 /* enable global interrupts */

    /* Power-on self test: single 2kHz beep for 100ms on PD4.
     * Non-blocking: beep.once() only arms the counter; the tone is
     * actually produced by beep.update() inside the TIM1 1ms ISR. */
    beep.once(&beep, 2, 100);

    uart.puts(&uart, "STM8S103 STEPPER ready\r\n");

    /* Event-driven main loop with buffered UART TX and 200ms pot sampling.
     * NOTE: 不能用 halt（TIM1 无法唤醒 halt）；用 wfi：CPU 暂停但外设时钟
     * 继续，任何中断（含 TIM1 1ms）唤醒后自动进入 ISR。 */
    while (1) {
        /* Drain pending TX bytes */
        uart.tx_service(&uart);

        /* UART RX echo */
        if (uart.rx_flag) {
            uart.rx_flag = 0;
            uart.putc(&uart, (char)uart.rx_byte);
        }

        /* ADC: sample PC4 every 200ms, map to ms/step and update speed.
         * Only print when the speed changes (avoid line spam). */
        if ((uint16_t)(g_tick - last_pot_tick) >= 200U) {
            uint16_t raw    = adc.read(&adc);
            uint16_t period = adc_to_period(raw);
            last_pot_tick = g_tick;
            if (period != last_period) {
                last_period = period;
                motor.setSpeed(&motor, period);
                uart.printf(&uart, "pot=%u spd=%u ms/step\r\n",
                            (unsigned int)raw, (unsigned int)period);
            }
        }

        __asm__("wfi");             /* wait-for-interrupt */
    }

    // return 0;
}
