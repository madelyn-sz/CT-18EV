/*
 * velo_kf.cpp
 *
 *  Created on: Oct 5, 2026
 *      Author: Madelyn
 */

#include "velo_kf.h"
#include "main.h"
#include "config.h"

#include <cstdlib>
#include <cstdint>
#include <algorithm>

enum calibration_status {
	CALIBRATION_IDLE,
	CALIBRATION_ACTIVE,
	CALIBRATION_SUCCEDED,
	CALIBRATION_FAILED
};

// all of these values are x1000 so they can be used immediately on accelerometer data
// before converting to floats
struct velo_kf_calibration {
	calibration_status flag_status = CALIBRATION_IDLE;

	volatile bool rtd_curr = false;
	volatile bool rtd_prev = false;
	volatile bool inputs_ok = false;

	int16_t min_reading = INT16_MAX;
	int16_t max_reading = INT16_MIN;

	int16_t offset_static = 0;
	int32_t offset_calibration_sum = 0;
	uint16_t offset_calibration_samples = 0;
};

struct velo_kf_state {
	int16_t accel_x_raw = 0;
};

static velo_kf_calibration c;
static velo_kf_state s;

// should also require motor rpm = 0

void upkeep_velo_kf_accel(uint8_t rtd, float tps, float bps) {
	c.inputs_ok = tps < TPS_THRESHOLD_OFF && bps > BPS_THRESHOLD_ON;
	c.rtd_curr = rtd;
}

void feed_velo_kf_accel(int16_t accel_x_raw) {
	bool rtd_rising = c.rtd_curr && !c.rtd_prev;

	c.rtd_prev = c.rtd_curr;

	if (!c.rtd_curr) {
		c.flag_status = CALIBRATION_IDLE;
	} else if (rtd_rising) {
		if (c.inputs_ok) {
			c.flag_status = CALIBRATION_ACTIVE;

			// don't reset offset (keep last good calibration), reset counters
			c.offset_calibration_sum = 0;
			c.offset_calibration_samples = 0;
			c.min_reading = INT16_MAX;
			c.max_reading = INT16_MIN;
		} else {
			c.flag_status = CALIBRATION_FAILED;
		}
	}

	if (c.flag_status == CALIBRATION_ACTIVE) {
		if (accel_x_raw > c.max_reading) c.max_reading = accel_x_raw;
		if (accel_x_raw < c.min_reading) c.min_reading = accel_x_raw;

		if (!c.inputs_ok || (c.max_reading - c.min_reading) > ACCEL_MAX_PLAUSIBLE_NOISE) {
			c.flag_status = CALIBRATION_FAILED;
		} else {
			c.offset_calibration_sum += accel_x_raw;
			c.offset_calibration_samples += 1;

			if (c.offset_calibration_samples >= OFFSET_CALIBRATION_SAMPLES) {
				int32_t candidate_offset =  c.offset_calibration_sum / c.offset_calibration_samples;

				if (std::abs(candidate_offset) <= ACCEL_MAX_PLAUSIBLE_OFFSET) {
					c.flag_status = CALIBRATION_SUCCEDED;
					c.offset_static = candidate_offset;
				} else {
					c.flag_status = CALIBRATION_FAILED;
				}
			}
		}
	}

	s.accel_x_raw = static_cast<int16_t>(
		std::clamp<int32_t>(static_cast<int32_t>(accel_x_raw) - c.offset_static, INT16_MIN, INT16_MAX)
	);
}
