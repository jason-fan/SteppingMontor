/**
 * motor.h - STM8S 2-Phase 4-Wire Stepper Motor Driver (Object-Oriented)
 *
 * Drives a 2-phase 4-wire stepper motor with a software commutation
 * table (1-2 phase excitation = 8 half-steps per electrical cycle).
 * The four phase outputs (IN1..IN4) are ordinary GPIO push-pull pins,
 * typically feeding a L9110s/ULN2003 driver board coil inputs (A, B, /A, /B).
 *
 * Speed control: period_ms = milliseconds per step (set by the
 * application from the potentiometer ADC reading). The phase is
 * advanced by motor.onTime() driven from the TIM1 1ms time-slice ISR,
 * so no blocking delay is ever used.
 *
 * Power window: each step energizes the target phase pattern for
 * on_ms milliseconds (passed to Motor_Create, 4 ms empirical default),
 * then releases all four coils (L9110S inputs all 0) until the next
 * step boundary — coil current flows only inside the window, saving
 * power / reducing heat at slow speed. If on_ms >= period_ms the off
 * branch never fires and the motor is effectively always energized
 * (continuous mode).
 *
 * Usage:
 *   #include "motor.h"
 *   Motor_t motor = Motor_Create(ports[], pins[], 4U);
 *   motor.init(&motor);
 *   motor.setSpeed(&motor, 10);   // 10 ms per step
 *   motor.start(&motor);
 *   motor.onTime(&motor, g_tick); // from TIM1 ISR every 1ms
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* ========== Coil Power Window ==========
 * on_ms (Motor_Create parameter): milliseconds of coil energization
 * per step, before the outputs go to 0 and the coils stay released
 * until the next step. MOTOR_ON_MS below is the recommended default
 * (empirical 3 ms) for main.c — pass 4U to Motor_Create, or tune it.
 *
 * Valid range: 1..period_ms.
 *   - Keep it <= the smallest period used (MOTOR_PERIOD_MIN = 2 ms in
 *     main.c) to guarantee an off-window at every speed.
 *   - If on_ms >= period_ms the motor is effectively always
 *     energized (continuous mode, no off window at that speed).
 * Empirical default 3 ms: at 2ms/step it is always-on; at >=4ms/step
 * it pulses 4ms on / rest off (low-speed coils stay cool, less current). */
#define MOTOR_ON_MS  3U

/* ========== Motor Class Definition ========== */

typedef struct Motor Motor_t;

struct Motor {
    uint16_t port[4];       /* GPIO port base address of IN1..IN4 */
    uint8_t  pin[4];        /* GPIO pin number of IN1..IN4 (0-7)  */

    /* --- Public state --- */
    uint8_t  dir;           /* 0 = CW, 1 = CCW                    */
    uint8_t  running;       /* 1 = stepping, 0 = stopped (coils off) */
    uint16_t period_ms;     /* ms per step (speed, min 1)          */

    /* --- Internal state (managed by motor.c, do not touch) --- */
    uint8_t  step_index;    /* current phase index 0..7 (half-step) */
    uint16_t acc;           /* tick accumulator inside current step */
    uint16_t on_ms;         /* coil-on window per step (MOTOR_ON_MS) */

    /* --- Public methods --- */
    void    (*init)     (Motor_t *self);      /* GPIO push-pull outputs, coils off */
    void    (*onTime)   (Motor_t *self, uint16_t tick); /* 1ms periodic event */
    void    (*start)    (Motor_t *self);      /* begin stepping */
    void    (*stop)     (Motor_t *self);      /* stop, release all coils */
    void    (*toggleDir)(Motor_t *self);      /* flip rotation direction */
    void    (*setDir)   (Motor_t *self, uint8_t dir);
    void    (*setSpeed) (Motor_t *self, uint16_t period_ms);
    uint8_t (*isRunning)(Motor_t *self);
};

/* ========== Constructor ========== */

/**
 * Create a stepper motor object bound to four GPIO pins.
 * Uses static storage — safe for single-instance use on STM8S103.
 * The constructor does NOT configure GPIO; call init() first.
 *
 * @param ports  array of 4 GPIO port base addresses (e.g. 0x5005 for PB)
 * @param pins   array of 4 GPIO pin numbers (0-7)
 * @param on_ms  coil-on window per step in ms (see MOTOR_ON_MS, 1..period_ms)
 * @return       fully initialized Motor_t object
 */
Motor_t Motor_Create(const uint16_t *ports, const uint8_t *pins, uint16_t on_ms);

#endif /* MOTOR_H */
