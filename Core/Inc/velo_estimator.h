/*
 * velo_kf.h
 *
 *  Created on: Oct 5, 2026
 *      Author: Madelyn
 */

#ifndef INC_VELO_ESTIMATOR_H_
#define INC_VELO_ESTIMATOR_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// threshold for throttle not depressed
#define TPS_THRESHOLD_OFF 0.05f

// threshold for brake being pressed
#define BPS_THRESHOLD_ON 0.7f

// number of samples to collect before completing calibration
#define OFFSET_CALIBRATION_SAMPLES 50

// max accelerometer readings that could plausibly be explained by noise
#define ACCEL_MAX_PLAUSIBLE_NOISE 0.01f

// max plausible static offset
#define ACCEL_MAX_PLAUSIBLE_OFFSET 1000

// max offset we can read and assume we are on mostly level ground
// computed from m/s^2 on a 0.5deg slope
#define ACCEL_SLOPE_TOL_MS2_X1000 86

#define MOTOR_RPM_THRESHOLD_STILL 5
#define WHEEL_SPEED_THRESHOLD_STILL 1

void upkeep_velo_estimator_conditions(uint8_t rtd, float tps, uint32_t motor_rpm, uint32_t wheel_speed);
void feed_velo_estimator_accel(int16_t raw_accel_x);

#ifdef __cplusplus
}
#endif


#endif /* INC_VELO_ESTIMATOR_H_ */
