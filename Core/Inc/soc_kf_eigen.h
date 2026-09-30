/*
 * soc_kf_eigen.h
 *
 *  Created on: Sep 22, 2026
 *      Author: Madelyn
 */

#ifndef INC_SOC_KF_EIGEN_H_
#define INC_SOC_KF_EIGEN_H_


#ifdef __cplusplus
extern "C" {
#endif

// C bridge

struct soc_kf_state;

float get_soc(struct soc_kf_state* state);
float get_vt(struct soc_kf_state* state);

void will_eigen_compile(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_SOC_KF_EIGEN_H_ */
