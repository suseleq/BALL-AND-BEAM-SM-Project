/*
 * servo.h
 *
 *  Created on: Dec 21, 2024
 *      Author: Kuba
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#define SERVO_MAX_US		2500
#define SERVO_MIN_US		500
#define SERVO_NEUTRAL		100

#define SERVO_MAX_ANGLE		90
#define SERVO_MIN_ANGLE		-90



void servo_init(TIM_HandleTypeDef *, uint32_t);
void servo_set_angle(int);

#endif /* INC_SERVO_H_ */
