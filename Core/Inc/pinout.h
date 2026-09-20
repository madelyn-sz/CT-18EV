/**
 * @file    pinout.h
 * @brief   GPIO assignments. Configuration itself is owned by CubeMX
 *          (MX_GPIO_Init); keep this in step with the .ioc.
 */

#ifndef PINOUT_H
#define PINOUT_H

#include "main.h"

/* RTD button, active high, external pull-down. */
#define PIN_RTD_BUTTON_PORT GPIOB
#define PIN_RTD_BUTTON      GPIO_PIN_14

/* RTD dash indicator. */
#define PIN_RTD_LIGHT_PORT GPIOB
#define PIN_RTD_LIGHT      GPIO_PIN_1

/* RTD buzzer. */
#define PIN_RTD_BUZZER_PORT GPIOB
#define PIN_RTD_BUZZER      GPIO_PIN_2

/* Configured by MX_GPIO_Init but unused: PB0 (output, never written),
 * PB12/PB13 (inputs, never read). */

/* Each ADC has a single regular channel, so the handle identifies the sensor. */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

#define ADC_TPS1 (&hadc1) /* ADC1 channel 10 */
#define ADC_TPS2 (&hadc2) /* ADC2 channel 11 */
#define ADC_BPS  (&hadc3) /* ADC3 channel 12 */

#endif /* PINOUT_H */
