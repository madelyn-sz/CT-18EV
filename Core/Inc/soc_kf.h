/**
 * @file    soc_kf.h
 * @brief   Pack SoC via a 1-RC EKF. TELEMETRY ONLY - never gates the control
 *          path; Orion's SoC stays authoritative for regen.
 */

#ifndef SOC_KF_H
#define SOC_KF_H

#include <stdint.h>

#define SOC_KF_CAN_ID_STATE 0x558

#define SOC_KF_NS 100
#define SOC_KF_NP 4

/* 1 = Orion reports discharge positive. UNCONFIRMED - if SoC climbs under
 * acceleration, flip it. */
#define SOC_KF_IBAT_DISCHARGE_POSITIVE 1

#define SOC_KF_Q_Z    1e-10f
#define SOC_KF_Q_V1   1e-6f
#define SOC_KF_R_MEAS 1e-3f
/* Orion seeds are only good to ~10%; the OCV figure is too confident for one. */
#define SOC_KF_P0_Z_OCV   1e-4f
#define SOC_KF_P0_Z_ORION 1e-2f
#define SOC_KF_P0_V1      1e-4f

#define SOC_KF_STALE_MS        200 // 10 missed 0x600 frames at 50 Hz
#define SOC_KF_REST_CURRENT_A  2.0f
#define SOC_KF_REST_MS         2000
#define SOC_KF_INIT_TIMEOUT_MS 3000
#define SOC_KF_P_FLOOR         1e-12f // stops the filter locking shut

/* Out-of-range readings skip the correction, else a corrupt frame is chased. */
#define SOC_KF_VCELL_MIN 2.0f
#define SOC_KF_VCELL_MAX 4.35f

#define SOC_KF_FLAG_INIT     0x01
#define SOC_KF_FLAG_OCV_INIT 0x02 // seeded from rest OCV, not Orion
#define SOC_KF_FLAG_BMS_LIVE 0x04
/* 0x08 was INV_LIVE; left vacant so the bits below keep their meaning. */
#define SOC_KF_FLAG_FROZEN  0x10 // input stale, estimate held
#define SOC_KF_FLAG_CLAMPED 0x20
#define SOC_KF_FLAG_VBAD    0x40

typedef struct {
    float soc;
    float v1;         // RC branch voltage, cell (V)
    float innovation; // V_meas - V_predicted, cell (V)
    float p00;
    float p11;
    float i_pack; // discharge-positive (A)
    float v_pack;
    float charge_ah;
    uint8_t temp_c;
    uint8_t orion_soc;
    uint8_t flags;
} soc_kf_debug_t;

void soc_kf_init(void);

/* Fed from the CAN RX ISR; tick_ms is HAL_GetTick(). */
void soc_kf_feed_bms(int16_t ibat_raw, uint16_t vbat_raw, uint8_t btmp_raw, uint8_t soc_raw,
                     uint32_t tick_ms);

/* Predict + correct. Main loop, ~11 Hz. */
void soc_kf_update(uint32_t tick_ms);

const soc_kf_debug_t *soc_kf_get_debug(void);

/* Writes bytes 0-6. Current/voltage/temp and Orion SoC are not repeated here -
 * the logger already decodes them from 0x600. Byte 7 belongs to the caller and
 * is only zeroed here; main.c puts VCU loop health in it. */
void soc_kf_pack_state(uint8_t *d);

#endif /* SOC_KF_H */
