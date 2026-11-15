/**
 * motor.c - STM8S 2-Phase 4-Wire Stepper Motor Driver Implementation
 *
 * Commutation table: 1-2 phase excitation (half-step), 8 phases.
 * Coil wiring: IN1 -> A phase, IN2 -> B phase, IN3 -> /A, IN4 -> /B.
 *         A+  B+  A-  B-
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
 * BIT-ORDER NOTE (mirror, verified on hardware): the table above is
 * written with bit3 = IN1 .. bit0 = IN4, while motor_write_phase()
 * writes pattern bit i to pin[i], i.e. bit0 = IN1 .. bit3 = IN4.
 * The two bit orders are mirrored, therefore the physical excitation
 * sequence is the REVERSE of the rows listed above, and dir = 0
 * (nominal CW) turns the motor in the opposite direction to the
 * labels here. This is expected behaviour, not a wiring bug — swap
 * the two wires of one winding (A-OA/A-OB or B-OA/B-OB) to invert.
 * Pin mapping used by main.c: IN1=PC3(A+), IN2=PC7(B+),
 * IN3=PC5(A-), IN4=PC6(B-); winding A = IN1+IN3, winding B = IN2+IN4.
 *
 * Timing (two-phase state machine): every step period is split into an
 * ON phase (coils energized, state_ms loaded with pw) and an OFF phase
 * (all four outputs 0, state_ms loaded with period_ms - pw). Each 1 ms
 * tick first consumes one millisecond from state_ms; only when
 * state_ms reaches 0 does the machine take the next transition. So
 * period_ms is the FULL step period including the coil-on window:
 * pw + (period_ms - pw) == period_ms and the commutation cadence is
 * unchanged. Fully non-blocking, driven by the 1ms time-slice tick
 * passed to onTime().
 *
 * Jog (stepOnce) performs no port I/O at call time: it only bumps
 * next_step (saturating). onTime() then pushes that step through
 * exactly the same ON/OFF countdowns as continuous motion, so a jog
 * step gets the same coil-on time as a running step, and once it is
 * done the machine parks in OFF with all coils released.
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

/* Per-phase power window: double-coil excitation draws more current and heats
 * faster than single-coil, so its on-time is shortened.  Index into this array
 * follows the half-step table order (A, A+B, B, /A+B, /A, /A+/B, /B, A+/B). */
static const uint8_t step_double[] = { 0, 1, 0, 1, 0, 1, 0, 1 };

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

/* Coil-on window (pw) of the phase currently sitting on the coils.
 *   single-coil phase -> on_ms
 *   double-coil phase -> (on_ms + 1) / 2  (two windings draw twice the
 *                        current, so the window is halved, rounded up)
 *
 * Overflow / truncation guards (state_ms is uint16_t, same width as pw):
 *   - on_ms is clamped to [1, MOTOR_PERIOD_LIMIT] by Motor_Create, so
 *     (on_ms + 1U) can never wrap around and the >>1 result always fits
 *     in uint16_t — no uint8_t narrowing, no lost bit.
 *   - step_index is always 0..7 (mod PHASE_STEPS), so step_double[] is
 *     never indexed out of bounds.
 *   - pw is floored at 1 ms: an ON phase of 0 ms would end on the same
 *     tick it started and the state machine would spin without ever
 *     energizing the coils (relevant if on_ms is ever passed as 0). */
static uint16_t motor_phase_pw(const Motor_t *self)
{
    uint16_t pw;

    if (step_double[self->step_index]) {
        pw = (uint16_t)((self->on_ms + 1U) >> 1);   /* ceil(on_ms / 2) */
    } else {
        pw = self->on_ms;
    }
    if (pw < 1U) {
        pw = 1U;                    /* on_ms == 0: keep a 1 ms window      */
    }
    if (pw > MOTOR_PERIOD_LIMIT) {
        pw = MOTOR_PERIOD_LIMIT;    /* same ceiling as period_ms           */
    }
    return pw;
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
    self->running    = 0;
    /* Deterministic power-on state: OFF phase with an expired countdown
     * and an empty step queue, so the machine stays parked with the
     * coils released until start() or stepOnce() is called — no phantom
     * step at boot and no coil left energized from a previous run. */
    self->state      = MOTOR_STATE_OFF;
    self->state_ms   = 0;
    self->next_step  = 0;
}

static void motor_onTime(Motor_t *self, uint16_t tick)
{
    uint16_t pw;
    uint16_t off_ms;

    (void)tick;

    /* Parked fast path: OFF with an expired countdown and nothing
     * queued. The coils are already released, so there is literally
     * nothing to do until start() or stepOnce() changes the state. */
    if (self->state_ms == 0U
        && self->state == MOTOR_STATE_OFF
        && self->next_step == 0U) {
        return;
    }

    /* ---- 1) Consume one millisecond from the current phase ----
     * The tick that loads state_ms does not decrement it (that would
     * shorten the phase by 1 ms), so a phase loaded with N ms expires on
     * exactly the N-th tick after it was started. A tick that takes the
     * countdown to 0 falls through to the transition below. */
    if (self->state_ms > 0U) {
        self->state_ms--;
        if (self->state_ms > 0U) {
            return;                    /* phase still running */
        }
    }

    /* ---- 2) ON phase expired: release the coils, load the OFF phase ---- */
    if (self->state == MOTOR_STATE_ON) {
        motor_write_phase(self, 0);    /* all four outputs 0: coils off */
        self->state = MOTOR_STATE_OFF;

        /* off_ms = period_ms - pw, i.e. the remainder of the step period.
         * Unsigned-underflow guard: setSpeed() clamps period_ms to
         * [1, MOTOR_PERIOD_LIMIT] (1000) and pw can legitimately be
         * >= period_ms (on_ms raised above the smallest period, or a clamp
         * at the fastest speed), so a plain subtraction would wrap to a
         * huge value (up to 65535 ms) and stall the motor for a minute.
         * Clamp to 1 ms instead: the step then degrades into back-to-back
         * ON windows, never into a stuck state machine.
         * Slowest step: period_ms = 1000, pw = 4 -> off_ms = 996 (or 998 on
         * the double-coil phases, pw = 2), still far below the uint16_t
         * ceiling (65535). */
        pw = motor_phase_pw(self);
        if (self->period_ms > pw) {
            off_ms = (uint16_t)(self->period_ms - pw);
        } else {
            off_ms = 1U;
        }
        self->state_ms = off_ms;

        /* Continuous motion never runs out of steps: re-arm the request
         * as soon as the OFF phase starts, so the countdown below always
         * finds one and the commutation cadence stays exactly
         * pw + off_ms == period_ms. Jog mode (running == 0) is left
         * alone: its queue drains and the machine parks. */
        if (self->running && self->next_step == 0U) {
            self->next_step = 1U;
        }
        return;
    }

    /* ---- 3) OFF phase expired: start the next step, if one is queued ----
     * Exactly one queued step is consumed per OFF expiry. In jog mode
     * next_step is non-zero only after a click, so an empty queue parks
     * the machine in OFF with the coils released (one click = one step). */
    if (self->next_step > 0U) {
        self->next_step--;                 /* consume one request      */
        motor_step_once(self);             /* advance phase, coils on  */
        self->state    = MOTOR_STATE_ON;
        self->state_ms = motor_phase_pw(self);  /* pw of the NEW phase */
        return;
    }

    /* Defensive: empty queue in OFF — stay parked, coils released. */
    self->state    = MOTOR_STATE_OFF;
    self->state_ms = 0U;
}

static void motor_start(Motor_t *self)
{
    /* Fixed starting point, same as the pre-refactor behaviour:
     *   - the CURRENT phase (step_index) is energized right away for its
     *     full pw, so starting never advances an extra step,
     *   - the countdown is loaded fresh, so no half-consumed window from a
     *     previous run survives a restart,
     *   - next_step stays armed at 1, so the OFF phase that follows always
     *     finds a request and the commutation cadence becomes exactly
     *     pw + (period_ms - pw) == period_ms per step. */
    self->running   = 1;
    self->next_step = 1;
    self->state     = MOTOR_STATE_ON;
    self->state_ms  = motor_phase_pw(self);   /* window of the current phase */
    motor_write_phase(self, phase_table[self->step_index]);
}

static void motor_stop(Motor_t *self)
{
    self->running   = 0;
    self->state     = MOTOR_STATE_OFF;
    self->state_ms  = 0;      /* no half-consumed phase survives a stop */
    self->next_step = 0;      /* drop queued steps: no extra step after stop */
    motor_write_phase(self, 0);   /* release all coils */
}

/* Jog: request exactly one step in the current direction. Only a counter is
 * bumped here — no port I/O, no phase change, so this is safe to call from
 * the key ISR. motor_onTime() consumes the request at the next OFF expiry
 * and drives it through the same ON/OFF countdowns as continuous motion
 * (that is what makes the per-step coil-on time identical), and parks in
 * OFF with the coils released once the step is finished.
 * The counter saturates at 0xFF, so a burst of fast clicks queues one step
 * each instead of wrapping to 0 and silently losing a click. */
static void motor_step_jog(Motor_t *self)
{
    if (self->next_step < 0xFFU) {
        self->next_step++;
    }
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
    /* Clamp to [1, MOTOR_PERIOD_LIMIT] (1000) ms per step — protects the
     * countdown arithmetic and keeps the motor from stalling at absurd
     * speeds. The upper bound must stay >= the slowest period the caller
     * asks for (1000 ms/step from the exponential pot curve in main.c),
     * otherwise setSpeed() would silently truncate it to a faster step.
     * A new period only takes effect at the next phase transition: the
     * phase already in flight keeps its own state_ms countdown, so changing
     * speed while running cannot truncate the current ON window (a shared
     * accumulator could, when the period was lowered below the elapsed
     * ticks). */
    if (period_ms < 1)                    period_ms = 1;
    if (period_ms > MOTOR_PERIOD_LIMIT)   period_ms = MOTOR_PERIOD_LIMIT;

    /* ---- Critical section: publish period_ms atomically ----
     * Why: period_ms is 16 bits but the STM8 core is 8-bit, so the store
     * below is TWO consecutive byte stores (STM8 is big-endian: high byte
     * first). motor_onTime() runs in the TIM1 1 ms ISR and reads period_ms
     * to compute off_ms = period_ms - pw. If a TIM1 tick lands between the
     * two byte stores, the ISR can latch a MIXED value (new high byte with
     * the old low byte, e.g. 0x03E8 -> 0x03FF) and load a garbage countdown
     * for that phase: one wrong step period, and at worst an off-window
     * stretched to ~1 s or truncated to 1 ms.
     * Scope: the clamps above are pure arithmetic on the local copy (no
     * shared state), so they stay OUTSIDE the window. The window is one
     * store wide - as short as it can be.
     * Caller contract: "rim" here UNCONDITIONALLY re-enables interrupts.
     * Every current caller runs either in the main loop (interrupts on) or
     * during boot before the final "rim" (in which case main.c re-closes
     * with an explicit "sim" right after this call, because tim1_setup()
     * has already set TIM1_IER.UIE by then). If setSpeed() is ever called
     * FROM an ISR or from a context that already executed "sim", this
     * sim/rim pair would exit with interrupts enabled and silently break
     * the caller's atomicity - such a caller must use a separate
     * "setSpeed_noirq" style variant that assumes interrupts are already
     * disabled, instead of calling this one. */
    __asm__("sim");
    self->period_ms = period_ms;
    __asm__("rim");
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
    motor.step_index = 0;
    motor.state      = MOTOR_STATE_OFF;
    motor.state_ms   = 0;
    motor.next_step  = 0;

    /* Clamp the coil-on window the same way setSpeed() clamps the period.
     * Motor_Create is the only writer of on_ms, so clamping here is what
     * guarantees (on_ms + 1U) can never wrap around inside
     * motor_phase_pw() and that pw always stays <= MOTOR_PERIOD_LIMIT. */
    if (on_ms < 1U)                    on_ms = 1U;
    if (on_ms > MOTOR_PERIOD_LIMIT)    on_ms = MOTOR_PERIOD_LIMIT;
    motor.on_ms = on_ms;              /* power-on window per step (from caller) */

    /* Bind methods */
    motor.init      = motor_init;
    motor.onTime    = motor_onTime;
    motor.start     = motor_start;
    motor.stop      = motor_stop;
    motor.stepOnce  = motor_step_jog;
    motor.toggleDir = motor_toggle_dir;
    motor.setDir    = motor_set_dir;
    motor.setSpeed  = motor_set_speed;
    motor.isRunning = motor_is_running;

    return motor;
}
