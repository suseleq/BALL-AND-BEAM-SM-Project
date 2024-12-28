/*
 * hc_sr04.h
 *
 *  Created on: Dec 23, 2024
 *      Author: Kuba
 */

#ifndef INC_HC_SR04_H_
#define INC_HC_SR04_H_

#include "stm32f7xx_hal.h"

struct us_sensor_str
{
    TIM_HandleTypeDef *htim_echo;  // Timer handle for echo signal
    TIM_HandleTypeDef *htim_trig;  // Timer handle for trigger signal
    uint32_t trig_channel;         // PWM trigger channel

    float distance_cm; // Distance in centimeters
};

// Initialize the HC-SR04 sensor
void hc_sr04_init(struct us_sensor_str *us_sensor, TIM_HandleTypeDef *htim_echo, TIM_HandleTypeDef *htim_trig, uint32_t channel);

// Convert time in microseconds to distance in centimeters
float hc_sr04_convert_us_to_cm(uint32_t distance_us);

#endif /* INC_HC_SR04_H_ */
