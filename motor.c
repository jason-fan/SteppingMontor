/**
 * motor.c - STM8S 2-Phase 4-Wire Stepper Motor Driver Implementation
 *
 * Commutation table: 1-2 phase excitation (half-step), 8 phases.
 * Coil wiring: IN1 -> A phase, IN2 -> B phase, IN3 -> /A, IN4 -> /B.
 *
 *   idx  IN1 IN2 IN3 IN4   active coils
 *   0     1   0   0   0    A
 *   1     1   1   0   0    A + B
 *   2     0   1   0   0    B
 *   3     0   1   1   0    /A + B
 *   4     0   0   1   0    /A
 *   5     0   0   1   1    /A + /B
 *   6     0   0   0   1    /B
 *   7     1   0   0   1    A + /B
 *
 * Forward (CW) walks the table upward, reverse (CCW) walks downward.
 * Stopping releases all coils (all outputs 0) to avoid coil heating.
 *
 * Timing: at each step boundary (every period_ms) the next phase is
 * written and the coils are energized for on_ms = MOTOR_ON_MS. When
 * on_ms elapses before the next step, all four outputs go to 0 and
 * the coils stay released until the next step boundary. Fully
 * non-blocking, counted by the 1ms time-slice tick passed to onTime().
 */

#include <stdint.h>
#include "motor.h"

/* ========== GPIO Register Access ========== */

#define GPIO_REG(port, off)     (*(volatile uint8_t *)((port) + (off)))
#define GPIO_ODR(port)          GPIO_REG(port, 0)
#define GPIO_DDR(port)          GPIO_REG(port, 2)
#define GPIO_CR1(port)          GPIO_REG(port, 3)
#define GPIO_CR2(port)          GPIO_REG(port, 4)

/* ========== Half-Step Phase Table (1-2 phase excitation) ========== */

static const uint8_t phase_table[8] = {
    0x08,  /* A        */
    0x0C,  /* A + B    */
    0x04,  /* B        */
    0x06,  /* /A + B   */
    0x02,  /* /A       */
    0x03,  /* /A + /B  */
    0x01,  /* /B       */
    0x09,  /* A + /B   */
};

#define PHASE_STEPS 8

/* ========== Internal Helpers ========== */

/* Write one phase pattern to the four coil pins.
 * Each pin is written with a read-modify-write on its own port, so
 * pins sharing a port (e.g. PC3/PC5/PC6/PC7) are still safe. */
static void motor_write_phase(Motor_t *self, uint8_t pattern)
{
    uint8_t i;
    for (i = 0; i < 4; i++) {
        uint16_t port = self->port[i];
        uint8_t  mask = (uint8_t)(1 << self->pin[i]);
        if (pattern & ((uint8_t)(1 << i))) {
            GPIO_ODR(port) |= mask;
        } else {
            GPIO_ODR(port) &= (uint8_t)~mask;
        }
    }
}

/* Advance one step: increment/decrement the phase index depending on
 * direction (CW = upward, CCW = downward through the 8-step table). */
static void motor_step_once(Motor_t *self)
{
    if (self->dir) {
        self->step_index = (uint8_t)((self->step_index + PHASE_STEPS - 1) % PHASE_STEPS);
    } else {
        self->step_index = (uint8_t)((self->step_index + 1) % PHASE_STEPS);
    }
    motor_write_phase(self, phase_table[self->step_index]);
}

/* ========== Motor Method Implementations ========== */

static void motor_init(Motor_t *self)
{
    uint8_t i;
    for (i = 0; i < 4; i++) {
        uint16_t port = self->port[i];
        uint8_t  mask = (uint8_t)(1 << self->pin[i]);
        GPIO_DDR(port) |= mask;   /* output                 */
        GPIO_CR1(port) |= mask;   /* push-pull              */
        GPIO_CR2(port) &= (uint8_t)~mask; /* 2MHz slew, default */
        GPIO_ODR(port) &= (uint8_t)~mask; /* coil off          */
    }
    self->step_index = 0;
    self->acc        = 0;
    self->running    = 0;
}

static void motor_onTime(Motor_t *self, uint16_t tick)
{
    (void)tick;
    if (!self->running) {
        return;
    }
    /* Step timeline (acc = ms elapsed inside current step):
     *   acc=0           step boundary: advance phase + energize coils
     *   acc=on_ms       power window over: release all coils (off)
     *   acc=period_ms   next step: wrap to 0 and re-energize
     * period_ms >= 1 guaranteed by setSpeed clamp; on_ms from MOTOR_ON_MS.
     * If on_ms >= period_ms the off branch never fires: the motor is
     * effectively always energized (continuous mode). */
    if (++self->acc >= self->period_ms) {
        self->acc = 0;
        motor_step_once(self);            /* new phase, coils on */
    } else if (self->on_ms < self->period_ms && self->acc == self->on_ms) {
        motor_write_phase(self, 0);       /* power window over: coils off */
    }
}

static void motor_start(Motor_t *self)
{
    self->acc     = 0;
    self->running = 1;
    /* Refresh the current phase on the pins (in case we were stopped
     * with coils released). */
    motor_write_phase(self, phase_table[self->step_index]);
}

static void motor_stop(Motor_t *self)
{
    self->running = 0;
    motor_write_phase(self, 0);   /* release all coils */
}

static void motor_toggle_dir(Motor_t *self)
{
    self->dir = (uint8_t)(self->dir ^ 1);
}

static void motor_set_dir(Motor_t *self, uint8_t dir)
{
    self->dir = (dir) ? 1 : 0;
}

static void motor_set_speed(Motor_t *self, uint16_t period_ms)
{
    /* Clamp to [1, 250] ms per step — protects the accumulator
     * arithmetic and keeps the motor from stalling at absurd speeds. */
    if (period_ms < 1)       period_ms = 1;
    if (period_ms > 250)     period_ms = 250;
    self->period_ms = period_ms;
}

static uint8_t motor_is_running(Motor_t *self)
{
    return self->running;
}

/* ========== Constructor ========== */

Motor_t Motor_Create(const uint16_t *ports, const uint8_t *pins, uint16_t on_ms)
{
    Motor_t motor;
    uint8_t i;

    for (i = 0; i < 4; i++) {
        motor.port[i] = ports[i];
        motor.pin[i]  = pins[i];
    }

    motor.dir        = 0;
    motor.running    = 0;
    motor.period_ms  = 10;   /* default 10 ms per step */
    motor.on_ms      = on_ms;         /* power-on window per step (from caller) */
    motor.step_index = 0;
    motor.acc        = 0;

    /* Bind methods */
    motor.init      = motor_init;
    motor.onTime    = motor_onTime;
    motor.start     = motor_start;
    motor.stop      = motor_stop;
    motor.toggleDir = motor_toggle_dir;
    motor.setDir    = motor_set_dir;
    motor.setSpeed  = motor_set_speed;
    motor.isRunning = motor_is_running;

    return motor;
}
