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
 * Step model (two-phase state machine): period_ms is the FULL step
 * period, coil-on window INCLUDED — it is not an idle/off duration.
 * Every step is split into two timed phases, each holding its own
 * remaining-millisecond countdown in state_ms:
 *
 *   MOTOR_STATE_ON  coils energized, state_ms counts down pw ms, where
 *                   pw = on_ms for single-coil phases (A / B / /A / /B)
 *                   pw = (on_ms+1)/2 for double-coil phases (A+B etc.)
 *   MOTOR_STATE_OFF all four coils released (L9110S inputs all 0),
 *                   state_ms counts down (period_ms - pw) ms until the
 *                   next step boundary
 *
 * Transitions happen only when a countdown reaches 0:
 *   ON  -> OFF : outputs forced to 0, OFF countdown = period_ms - pw
 *   OFF -> ON  : one step consumed from next_step, phase advanced,
 *                coils energized, ON countdown = pw of the new phase
 *   OFF, next_step == 0 : park in OFF, coils stay released (jog idle)
 *
 * Because pw + (period_ms - pw) == period_ms, the commutation cadence
 * and the duty cycle are identical to the old accumulator scheme
 * (4 ms on / 2 ms off at the 6 ms fastest step of main.c). Coil
 * current flows only inside the ON window, saving power and reducing
 * heat at slow speed.
 *
 * Usage:
 *   #include "motor.h"
 *   Motor_t motor = Motor_Create(ports[], pins[], 4U);
 *   motor.init(&motor);
 *   motor.setSpeed(&motor, 10);   // 10 ms per step
 *   motor.start(&motor);
 *   motor.stepOnce(&motor);       // jog: queue one step in next_step;
 *                                 // onTime() drives it through the same
 *                                 // ON/OFF countdowns as a run step and
 *                                 // parks in OFF (coils released) after it
 *   motor.onTime(&motor, g_tick); // from TIM1 ISR every 1ms
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* ========== Coil Power Window ==========
 * on_ms (Motor_Create parameter): milliseconds of coil energization
 * inside every step period. period_ms is the WHOLE period (coil-on
 * window included), so the released part of a step is
 * (period_ms - pw) ms, not the whole period. MOTOR_ON_MS below is the
 * default (4 ms) for main.c — pass MOTOR_ON_MS, or tune it.
 *
 * Valid range: 1..period_ms.
 *   - Keep it <= the smallest period used (MOTOR_PERIOD_MIN = 6 ms in
 *     main.c) to guarantee an off-window at every speed.
 *   - If on_ms >= period_ms the off-window would collapse to 0; motor.c
 *     clamps it to the minimum of 1 ms so the countdown never stalls.
 * 4 ms: at the 6 ms fastest step it pulses 4 ms on / 2 ms off, and the
 * off-window grows with slower steps (e.g. 4 ms on / 996 ms off at the
 * 1000 ms slowest step of the exponential pot curve; the double-coil
 * phases use pw = 2 ms, so their off-window is 998 ms), so the coils are
 * never held on continuously. */
#define MOTOR_ON_MS  4U

/* ========== Two-Phase State Machine ==========
 * state holds one of these two values; state_ms holds the milliseconds
 * still to run in the current phase. state_ms == 0 means "phase is due
 * to end", which motor_onTime() resolves into the next transition. */
#define MOTOR_STATE_OFF   0U   /* coils released, waiting for next step  */
#define MOTOR_STATE_ON    1U   /* coils energized for the power window   */

/* Clamp limit shared by setSpeed() and Motor_Create(): period_ms and
 * on_ms never exceed this, and state_ms is a uint16_t, so the remaining
 * countdown can hold the widest period with no truncation.
 * Raised 250 -> 1000 together with the exponential pot curve in main.c
 * (slowest step = 1000 ms/step): state_ms must be able to hold the whole
 * 1000 ms period AND the derived off-window (period_ms - pw, up to 998 ms
 * when pw = 2 ms), which uint16_t (max 65535) holds with huge margin. */
#define MOTOR_PERIOD_LIMIT  1000U

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
    uint8_t  state;         /* MOTOR_STATE_ON / MOTOR_STATE_OFF: which timed
                             * phase of the step we are in                */
    uint16_t state_ms;      /* ms still left in the current phase; 0 = the
                             * phase has just expired and the transition is
                             * taken on the next 1ms tick. uint16_t covers
                             * MOTOR_PERIOD_LIMIT (1000) and the widest
                             * off-window (998 ms) with no truncation      */
    uint16_t on_ms;         /* coil-on window per step (MOTOR_ON_MS) */
    uint8_t  next_step;     /* pending step requests. Continuous run keeps it
                             * at 1 (never exhausted); jog adds one per
                             * stepOnce() click and saturates at 0xFF      */

    /* --- Public methods --- */
    void    (*init)     (Motor_t *self);      /* GPIO push-pull outputs, coils off */
    void    (*onTime)   (Motor_t *self, uint16_t tick); /* 1ms periodic event */
    void    (*start)    (Motor_t *self);      /* begin stepping */
    void    (*stop)     (Motor_t *self);      /* stop, release all coils */
    void    (*stepOnce) (Motor_t *self);      /* jog: queue 1 step, run by onTime() */
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
 * @param on_ms  coil-on window per step in ms (see MOTOR_ON_MS, 1..MOTOR_PERIOD_LIMIT)
 * @return       fully initialized Motor_t object
 */
Motor_t Motor_Create(const uint16_t *ports, const uint8_t *pins, uint16_t on_ms);

#endif /* MOTOR_H */
