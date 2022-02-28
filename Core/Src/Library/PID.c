/*
 * PID.c
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */
#include <Library/PID.h>

void PIDController_initialise(PIDController *pid, double Kp, double Ki,
		double Kd) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->out = 0;
}

void PIDController_set_limit(PIDController *pid, double int_lim, double out_lim) {
	if (out_lim != 0) {
		pid->out_lim = out_lim;
		pid->out_lim_frag = 1;
	}
	if (int_lim != 0) {
		pid->int_lim = int_lim;
		pid->int_lim_frag = 1;
	}
}

double PIDController_update(PIDController *pid, double setpoint,
		double measurement) {
	double error = setpoint - measurement;
	/*
	 * P term
	 */
	pid->proportional_term = pid->Kp * error;

	/*
	 * I term
	 */
	pid->integrator += error;
	pid->integral_term = pid->Ki * pid->integrator;
	if (pid->int_lim_frag){
		if (pid->integral_term > pid->int_lim){
			pid->integral_term = pid->int_lim;
		}
		else if (pid->integral_term < (-1*pid->int_lim)){
			pid->integral_term = (pid->int_lim * -1);
		}
	}
	/*
	 * D term
	 */
	pid->derivative_term = pid->Kd * (error - pid->prevError);
	pid->prevError = error;
	/*
	 * Calculate a final value
	 */
	pid->out = pid->proportional_term + pid->integral_term
			+ pid->derivative_term;
	if (pid->out_lim_frag){
		if (pid->out > pid->out_lim){
			pid->out = pid->out_lim;
		}
		else if (pid->out < (-1*pid->out_lim)){
			pid->out= (pid->out_lim * -1);
		}
	}
	return pid->out;
}

double Cascade_PIDController_update(PIDController *position_pid,
		PIDController *velocity_pid, KalmanFilter *kalman_filter,
		double desired_position, double desired_velocity) {
	double velocity_command = PIDController_update(position_pid,
			desired_position, kalman_filter->x1);
	double velocity_error = desired_velocity + velocity_command
			- kalman_filter->x2;
	double out = PIDController_update(velocity_pid, velocity_error,
			kalman_filter->x2);
	return out;
}
