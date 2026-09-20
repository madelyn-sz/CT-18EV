/**
 * @file    regen.h
 * @brief   Regenerative braking torque on a closed throttle.
 *
 * While coasting above the cutoff with a healthy, non-full pack, the driver
 * request is replaced by a braking torque that ramps down to
 * REGEN_TORQUE_AT_CUTOFF as the car slows, so it does not step on dropout.
 * No HAL dependency, so it is testable on the host.
 */

#ifndef REGEN_H
#define REGEN_H

#include <stdint.h>

#define REGEN_TPS_THRESHOLD 0.05f

/* Regen releases below this motor speed */
#define REGEN_CUTOFF_RPM 500u

/* Below this entry speed there is not enough range above the cutoff for a
 * useful ramp */
#define REGEN_MIN_RAMP_RPM 550u

/* Regen is inhibited above this pack SoC */
#define REGEN_MAX_SOC_PERCENT 80u

/* Braking torque, Nm x10, always negative. Effort is highest at the cutoff
 * speed and lowest at the speed regen engaged. Formerly MIN_REGEN_TORQUE and
 * MAX_REGEN_TORQUE in main.c. */
#define REGEN_TORQUE_AT_ENTRY  (-200)
#define REGEN_TORQUE_AT_CUTOFF (-250)

typedef struct {
    uint8_t active;           /* 1 = regen is driving the torque request   */
    uint32_t entry_speed_rpm; /* highest speed seen this regen event       */
    int32_t torque;           /* last regen torque applied, Nm x10         */
} regen_debug_t;

void regen_init(void);

/* Returns the torque to command, Nm x10: negative while regen is active,
 * otherwise driver_torque unchanged. bms_live gates the soc_percent input. */
int32_t regen_update(int32_t driver_torque, uint32_t motor_speed_rpm, float tps_combined,
                     uint8_t brake_pressed, uint8_t soc_percent, uint8_t bms_live);

const regen_debug_t *regen_get_debug(void);

#endif /* REGEN_H */
