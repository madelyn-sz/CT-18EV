/*
 * launch_control_2.cpp
 *
 *  Created on: Oct 15, 2026
 *      Author: Madelyn
 */


#include "launch_control.h"
#include <stdint.h>

const uint32_t lc_takeup_end_ms = LC_TAKEUP_TIME_MS;
const uint32_t lc_limited_driveline_end_ms = lc_takeup_end_ms + LC_LIMITED_DRIVELINE_TIME_MS;
const uint32_t lc_limited_traction_end_ms = lc_limited_driveline_end_ms + LC_LIMITED_TRACTION_TIME_MS;

const int32_t takeup_end_torque = LC_TRACTION_LIMIT_TORQUE_NMX10 * LC_TAKEUP_TORQUE_PCT;
const int32_t limited_driveline_end_torque = takeup_end_torque + LC_LIMITED_DRIVELINE_RATE * (LC_LIMITED_DRIVELINE_TIME_MS/1000.0f);
const int32_t limited_traction_end_torque = limited_driveline_end_torque + LC_LIMITED_TRACTION_RATE * (LC_LIMITED_TRACTION_TIME_MS/1000.0f);
void lc_reset_pid(lc_data_t* data) {
	data->pid_p = 0;
	data->pid_i = 0;
	data->pid_d = 0;
}

// TODO: timeout for open loop hold?
// return torque limit as Nm X10
int32_t lc_update(lc_data_t* data, float tps) {
	switch (data->state) {
	case LC_STATE_ARMED:
		lc_reset_pid(data);

		// Bail out if throttle falls below throttle release limit and front wheel speed is below crossover
		if (
				tps_combined < LC_THROTTLE_RELEASE &&
				data->front_wheel_speed <= LC_CROSSOVER_SPEED_MS
		) {
			dbg.state = LC_STATE_IDLE;
		// Enter open loop launching if enabled and above throttle trigger limit
		} else if (
				launch_control_enable &&
				tps >= LC_THROTTLE_TRIGGER
		) {
				data->openloop.state = LC_OPENLOOP_TAKEUP;
				data->openloop.start_tick = HAL_GetTick();
				data->state = LC_STATE_LAUNCHING_OPENLOOP;
		}

		break;
	case LC_STATE_LAUNCHING_OPENLOOP:
		uint32_t tick_now = HAL_GetTick();
		uint32_t t = tick_now - data->openloop.tick_start;
		float torque_limit = 0;

		if (t < lc_takeup_end_ms) {
			// a constant small torque to take up backlash
			torque_limit = LC_TRACTION_LIMIT_TORQUE_NM * LC_TAKEUP_TORQUE_PCT;
		} else if (t < lc_limited_driveline_end_ms) {
			// fast linear ramp through driveline limited section
			torque_limit = LC_LIMITED_DRIVELINE_RATE * ((t - lc_takeup_end_ms)*0.001f) + takeup_end_torque;
		} else if (t < lc_limited_traction_end_ms) {
			// slow linear ramp through traction limited section
			torque_limit =
				LC_LIMITED_TRACTION_RATE * ((t - lc_limited_driveline_end_ms)*0.001f) + limited_driveline_end_torque;
		} else {
			if (data->closedloop.edges_seen >= LC_MIN_EDGES && data->closedloop.front_wheel_speed >= LC_CROSSOVER_SPEED_MS) {
				data->state = LC_STATE_LAUNCHING_CLOSEDLOOP;
			}

			torque_limit = LC_TRACTION_LIMIT_TORQUE_NM;
		}

		if (torque_limit > LC_TRACTION_LIMIT_TORQUE_NM) {
			return LC_TRACTION_LIMIT_TORQUE_NMX10;
		} else if (torque_limit < 0) {
			return 0;
		} else {
			return (int32_t)(torque_limit * 10);
		}

		break;
	case LC_STATE_LAUNCHING_CLOSEDLOOP:
		break;
	case LC_STATE_IDLE:
		break;
	}
}
