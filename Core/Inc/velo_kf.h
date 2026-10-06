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

void velo_kf_feed_accel(int16_t raw_accel_x);
void velo_kf_feed_encoder(uint16_t raw_encoder_l, uint16_t raw_encoder_r);

#ifdef __cplusplus
}
#endif


#endif /* INC_VELO_KF_H_ */
