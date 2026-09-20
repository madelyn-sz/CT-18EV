#include "launch_control.h"
#include <math.h>

/* Do not set to 1 until lc_map_rpm/lc_map_torque are populated. They are
 * placeholders: every breakpoint 0 rpm, every value 1.0 Nm, so the lookup
 * returns 1.0 Nm at any speed and the car is clamped to a crawl. */
volatile uint8_t launch_control_enable = 0;

#define LC_MAP_SIZE 8

// RPM vs Torque maps
static const uint16_t lc_map_rpm[LC_MAP_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};

// These are x10 values
static const uint16_t lc_map_torque[LC_MAP_SIZE] = {10, 10, 10, 10, 10, 10, 10, 10};

/* Interpolate a table of monotonically increasing breakpoints. Separate from
 * lc_map_lookup() so tests can drive it with a populated table. */
static uint32_t map_interp(const uint16_t *bp, const uint16_t *val, int n, uint32_t x)
{
    // Clamp to the ends of the table
    if (x <= bp[0]) {
        return val[0];
    }
    if (x >= bp[n - 1]) {
        return val[n - 1];
    }

    // Find the bracketing breakpoints and interpolate
    for (int i = 0; i < n - 1; i++) {
        if (x < bp[i + 1]) {
            const uint32_t bp_lo = bp[i];
            const uint32_t bp_hi = bp[i + 1];
            const uint32_t val_lo = val[i];
            const uint32_t val_hi = val[i + 1];
            const uint32_t bp_range = bp_hi - bp_lo;

            if (bp_range == 0) {
                return val_lo;
            }

            // Linear interpolation (integer math, no FP needed)
            const int32_t val_range = (int32_t)val_hi - (int32_t)val_lo;
            const uint32_t offset = x - bp_lo;
            return (uint32_t)((int32_t)val_lo + (val_range * (int32_t)offset) / (int32_t)bp_range);
        }
    }

    return val[n - 1];
}

static uint32_t lc_map_lookup(uint32_t motor_speed_rpm)
{
    return map_interp(lc_map_rpm, lc_map_torque, LC_MAP_SIZE, motor_speed_rpm);
}

// Now float (32-bit) — atomic on Cortex-M3, was volatile double (non-atomic bug)
static volatile float front_wheel_speed_ms = 0.0f;

// Timestamp of last AiM wheel speed CAN message
static volatile uint32_t last_wheel_speed_tick = 0;

// Current system tick, fed from main loop
static uint32_t current_tick_ms = 0;

// Debug/telemetry state exposed via lc_get_debug()
static lc_debug_t dbg;

static float prev_slip_ratio = 0.0f;

static float motor_rpm_to_wheel_ms(uint32_t motor_speed_rpm)
{
    return (float)motor_speed_rpm * LC_RPM_TO_MS;
}

static void lc_reset_pi(void)
{
    dbg.pi_i_term = 0.0f;
    dbg.pi_p_term = 0.0f;
    dbg.pi_output = 0.0f;
    dbg.slip_rate_cut = 0;
}

/* Clear on the transition, not on the next pass through the IDLE case, so the
 * published PI terms match the published state. */
static void lc_enter_idle(void)
{
    dbg.state = LC_STATE_IDLE;
    lc_reset_pi();
}

void lc_init(void)
{
    dbg.state = LC_STATE_IDLE;
    dbg.slip_ratio = 0.0f;
    dbg.slip_ratio_raw = 0.0f;
    dbg.slip_rate = 0.0f;
    dbg.pi_p_term = 0.0f;
    dbg.pi_i_term = 0.0f;
    dbg.pi_output = 0.0f;
    dbg.lc_torque = 0;
    dbg.map_torque = 0;
    dbg.vehicle_speed = 0.0f;
    dbg.rear_wheel_speed = 0.0f;
    dbg.sensor_healthy = 0;
    dbg.slip_rate_cut = 0;
    prev_slip_ratio = 0.0f;
}

void lc_feed_wheel_speed(uint16_t front_left_speed_x10, uint16_t front_right_speed_x10)
{
    // Convert from km/h × 10 to m/s using precomputed constant
    float left_ms = (float)front_left_speed_x10 * LC_KMH10_TO_MS;
    float right_ms = (float)front_right_speed_x10 * LC_KMH10_TO_MS;
    front_wheel_speed_ms = (left_ms + right_ms) * 0.5f;
    last_wheel_speed_tick = current_tick_ms;
}

void lc_feed_tick(uint32_t tick_ms)
{
    current_tick_ms = tick_ms;
}

const lc_debug_t *lc_get_debug(void)
{
    return &dbg;
}

lc_state_t lc_get_state(void)
{
    return dbg.state;
}

int32_t lc_update(int32_t driver_torque, uint32_t motor_speed_rpm, float tps_combined)
{

    uint8_t sensor_ok = 1;
    uint32_t elapsed = current_tick_ms - last_wheel_speed_tick;
    // Handle tick wraparound if elapsed is huge, sensor timed out
    if (elapsed > LC_SENSOR_TIMEOUT_MS || last_wheel_speed_tick == 0) {
        sensor_ok = 0;
    }
    dbg.sensor_healthy = sensor_ok;

    float v_rear = motor_rpm_to_wheel_ms(motor_speed_rpm);
    float v_front = front_wheel_speed_ms; // volatile float — atomic on CM3

    dbg.rear_wheel_speed = v_rear;
    dbg.vehicle_speed = v_front;

    // Slip ratio: lambda = (v_rear - v_front) / max(v_rear, v_front, epsilon)
    float max_speed = fmaxf(v_rear, v_front);
    float slip_raw = 0.0f;
    // maybe make this value bigger?
    if (max_speed > 0.5f) {

        slip_raw = (v_rear - v_front) / max_speed; // only when meaningful
    }
    slip_raw = fmaxf(0.0f, fminf(slip_raw, 1.0f));
    dbg.slip_ratio_raw = slip_raw;

    // Low-pass filter on slip ratio
    dbg.slip_ratio =
        dbg.slip_ratio * LC_SLIP_FILTER_ALPHA + slip_raw * (1.0f - LC_SLIP_FILTER_ALPHA);

    // Slip rate (dlambda/dt)
    dbg.slip_rate = (dbg.slip_ratio - prev_slip_ratio) / LC_DT_S;
    prev_slip_ratio = dbg.slip_ratio;

    switch (dbg.state) {

    case LC_STATE_IDLE:
        lc_reset_pi();

        if (launch_control_enable) {
            dbg.state = LC_STATE_ARMED;
        }
        break;

    case LC_STATE_ARMED:
        lc_reset_pi();
        prev_slip_ratio = 0.0f;

        if (!launch_control_enable) {
            lc_enter_idle();
        } else if (tps_combined >= LC_THROTTLE_TRIGGER) {
            dbg.state = LC_STATE_LAUNCHING_OPENLOOP;
        }
        break;

    case LC_STATE_LAUNCHING_OPENLOOP:

        if (!launch_control_enable || tps_combined < 0.05f) {
            lc_enter_idle();
        }
        // Crossover to closed loop when vehicle speed is reliable
        else if (sensor_ok && v_front > LC_CROSSOVER_SPEED_MS) {
            dbg.state = LC_STATE_LAUNCHING_CLOSEDLOOP;
        }
        break;

    case LC_STATE_LAUNCHING_CLOSEDLOOP:

        if (!launch_control_enable || tps_combined < 0.05f) {
            lc_enter_idle();
        } else if (v_front > LC_EXIT_SPEED_MS) {
            lc_enter_idle();
        }
        // Fall back to open-loop if sensor dies
        else if (!sensor_ok) {
            dbg.state = LC_STATE_LAUNCHING_OPENLOOP;
        }
        break;
    }

    /* LC only reduces positive drive torque. Regen is negative and passes
     * through, so the clamps below can assume a positive bound. */
    if (driver_torque <= 0) {
        dbg.map_torque = driver_torque;
        dbg.lc_torque = driver_torque;
        dbg.slip_rate_cut = 0;
        return driver_torque;
    }

    int32_t lc_torque_limit = driver_torque; // Default: passthrough

    switch (dbg.state) {

    case LC_STATE_IDLE:
    case LC_STATE_ARMED:

        dbg.map_torque = driver_torque;
        dbg.lc_torque = driver_torque;
        return driver_torque;

    case LC_STATE_LAUNCHING_OPENLOOP: {

        int32_t map_t = (int32_t)lc_map_lookup(motor_speed_rpm);
        dbg.map_torque = map_t;
        lc_torque_limit = map_t;

        // Even in open loop, if we have sensor data, apply slip rate limiter
        if (sensor_ok && dbg.slip_rate > LC_SLIP_RATE_MAX) {
            lc_torque_limit = (int32_t)((float)lc_torque_limit * LC_SLIP_RATE_CUT_MULT);
            dbg.slip_rate_cut = 1;
        } else {
            dbg.slip_rate_cut = 0;
        }
        break;
    }

    case LC_STATE_LAUNCHING_CLOSEDLOOP: {

        //  error = lambda_target - lambda_actual
        //  Positive error: below target slip → allow more torque
        //  Negative error: above target slip → reduce torque
        float error = LC_SLIP_TARGET - dbg.slip_ratio;

        dbg.pi_p_term = LC_KP * error;
        dbg.pi_i_term += LC_KI * error * LC_DT_S;

        // Clamping the I term to -driver_torque to driver_torque
        if (dbg.pi_i_term < -(float)driver_torque) {
            dbg.pi_i_term = -(float)driver_torque;
        }
        if (dbg.pi_i_term > (float)driver_torque) {
            dbg.pi_i_term = (float)driver_torque;
        }

        // Controller output
        dbg.pi_output = dbg.pi_p_term + dbg.pi_i_term;

        float cl_torque = (float)driver_torque + dbg.pi_output;

        // Clamp the torque values between zero and driver_torque
        if (cl_torque < 0.0f)
            cl_torque = 0.0f;
        if (cl_torque > (float)driver_torque)
            cl_torque = (float)driver_torque;

        lc_torque_limit = (int32_t)cl_torque;
        // look up from table based on motor speed.
        int32_t map_t = (int32_t)lc_map_lookup(motor_speed_rpm);
        dbg.map_torque = map_t;

        // Slip rate emergency override
        if (dbg.slip_rate > LC_SLIP_RATE_MAX) {
            lc_torque_limit = (int32_t)((float)lc_torque_limit * LC_SLIP_RATE_CUT_MULT);
            dbg.slip_rate_cut = 1;
        } else {
            dbg.slip_rate_cut = 0;
        }
        break;
    }
    }
    // clamps it to driver torque
    if (lc_torque_limit > driver_torque) {
        lc_torque_limit = driver_torque;
    }

    dbg.lc_torque = lc_torque_limit;
    return lc_torque_limit;
}
