/*
 * velo_kf.h
 *
 *  Created on: Oct 5, 2026
 *      Author: Madelyn
 */

#ifndef INC_VELO_KF_H_
#define INC_VELO_KF_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// theshold for throttle unpressed
#define TPS_THRESHOLD_OFF 0.05f

// threshold for brake being pressed
#define BPS_THRESHOLD_ON 0.7f

// number of samples to collect before completing calibration
#define OFFSET_CALIBRATION_SAMPLES 50

// max accelerometer readings that could plausibly be explained by noise
#define ACCEL_MAX_PLAUSIBLE_NOISE 0.01f

// max plausible static offset
#define ACCEL_MAX_PLAUSIBLE_OFFSET 1

void upkeep_velo_kf_accel(uint8_t rtd, float tps, uint32_t motor_rpm, uint32_t wheel_speed);
void feed_velo_kf_accel(int16_t raw_accel_x);

#ifdef __cplusplus
}
#endif


#endif /* INC_VELO_KF_H_ */
