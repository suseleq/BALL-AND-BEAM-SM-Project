/*
 * pid_controller.h
 *
 *  Created on: Dec 28, 2024
 *      Author: Kuba
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

typedef struct
{
	float prev_error;
	float total_error;
	float Kp;
	float Ki;
	float Kd;
	int max_u;
	int min_u;
	float fs;
	float anti_windup_limit;
	float prev_u;
}pid_str;

void pid_init(pid_str *pid_data, float kp_init, float ki_init, float kd_init, float fs_init, float anti_windup_limit_init);
int pid_calculate(pid_str *pid_data, float y_ref, float y);


#endif /* INC_PID_CONTROLLER_H_ */
