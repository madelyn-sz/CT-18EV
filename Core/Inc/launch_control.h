#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#include <stdint.h>

#define LC_GEAR_RATIO 3.636f
#define LC_TIRE_RADIUS_M 0.2f

#define LC_AIM_WHEEL_SPEED_CAN_ID 0x300
#define LC_DEBUG_CAN_ID 0x557

#define LC_SLIP_TARGET        0.10f
#define LC_CROSSOVER_SPEED_MS 2.0f
#define LC_EXIT_SPEED_MS      20.0f
#define LC_MIN_EDGES 3

/* Throttle thresholds */
#define LC_THROTTLE_TRIGGER 0.40f
#define LC_THROTTLE_RELEASE 0.05f

/* Closed-loop PID gains */
#define LC_KP 5000.0f
#define LC_KI 500.0f
#define LC_KD 330.0f

/* Open-loop slip rate cut */
#define LC_SLIP_RATE_MAX      3.0f
#define LC_SLIP_RATE_CUT_MULT 0.5f

/* Slip filter */
#define LC_SLIP_TAU_S        0.025f
#define LC_SLIP_FILTER_ALPHA 0.670320f

#define LC_SENSOR_TIMEOUT_MS 100
#define LC_DT_S 0.010f

/* Loop period bounds */
#define LC_DT_MIN_MS 1u
#define LC_DT_MAX_MS 100u
#define LC_DT_MIN_S (LC_DT_MIN_MS * 0.001f)
#define LC_DT_MAX_S (LC_DT_MAX_MS * 0.001f)

/* Conversions */
#define LC_RPM_TO_MS ((2.0f * 3.14159265f / 60.0f) * LC_TIRE_RADIUS_M / LC_GEAR_RATIO)
#define LC_MS_TO_RPM (LC_GEAR_RATIO * 60.0f / (2.0f * 3.14159265f * LC_TIRE_RADIUS_M))
#define LC_KMH10_TO_MS (1.0f / 36.0f)

// TODO: Estimate from weight transfer
#define LC_TRACTION_LIMIT_TORQUE_NM 220

// TODO: Tune!
#define LC_TAKEUP_TORQUE_PCT 0.1
#define LC_LIMITED_DRIVELINE_RATE 1
#define LC_LIMITED_TRACTION_RATE 1

#define LC_TAKEUP_TIME_MS 50
#define LC_LIMITED_DRIVELINE_TIME_MS 100
#define LC_LIMITED_TRACTIION_TIME_MS 50

typedef enum lc_state {
	LC_STATE_ARMED,
	LC_STATE_LAUNCHING_OPENLOOP,
	LC_STATE_LAUNCHING_CLOSEDLOOP,
	LC_STATE_IDLE
} lc_state_t;

typedef struct lc_debug {
	lc_state_t state;

	uint16_t edges_seen = 0;
	float front_wheel_speed = 0;
	float rear_wheel_speed = 0;
} lc_data_t;

#endif /* LAUNCH_CONTROL_H */
