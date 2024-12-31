/*
 * pid_controller.c
 *
 *  Created on: Dec 28, 2024
 *      Author: Kuba
 */
#include "pid_controller.h"

void pid_init(pid_str *pid_data, float kp_init, float ki_init, float kd_init, float fs_init, float anti_windup_limit_init) {
    pid_data->prev_error = 0.0f;
    pid_data->total_error = 0.0f;
    pid_data->max_u = 90;
    pid_data->min_u = -90;

    pid_data->Kp = kp_init;
    pid_data->Ki = ki_init;
    pid_data->Kd = kd_init;
    pid_data->fs = fs_init;

    pid_data->anti_windup_limit = anti_windup_limit_init;
}

int pid_calculate(pid_str *pid_data, float y_ref, float y) {
    float error = y_ref - y;

    if(error < 0.002f && error > -0.002f){
    	error = 0;
    }

    float p = pid_data->Kp * error;
    float i = pid_data->Ki * pid_data->total_error;
    float d = pid_data->Kd * (error - pid_data->prev_error);

    int u = p + i + d;

    //Saturation
    if (u > pid_data->max_u) {
        u = pid_data->max_u;
    } else if (u < pid_data->min_u) {
        u = pid_data->min_u;
    } else {
        pid_data->total_error += error / pid_data->fs;

        // Antiwindup
        if (pid_data->total_error > pid_data->anti_windup_limit) {
            pid_data->total_error = pid_data->anti_windup_limit;
        } else if (pid_data->total_error < -pid_data->anti_windup_limit) {
            pid_data->total_error = -pid_data->anti_windup_limit;
        }
    }


    pid_data->prev_error = error;

    return u;
}
