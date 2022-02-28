/*
 * PID.h
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <Library/KalmanFilter.h>

typedef struct {

	/* Controller gains */
	double Kp;
	double Ki;
	double Kd;

	/* Output limits */
	double out_lim;
	double out_lim_frag;

	/* Integrator limits */
	double int_lim;
	double int_lim_frag;

	/* Controller "memory" */
	double proportional_term;
	double integrator;
	double integral_term;
	double derivative_term;
	double prevError;		/* Required for differentiator */

	/* Controller output */
	double out;

} PIDController;

void PIDController_initialise(PIDController *pid, double Kp, double Ki, double Kd);
void PIDController_set_limit(PIDController *pid, double int_lim, double out_lim);
double PIDController_update(PIDController *pid, double setpoint, double measurement);
double Cascade_PIDController_update(PIDController *position_pid,
		PIDController *velocity_pid, KalmanFilter *kalman_filter, double desired_position,
		double desired_velocity);
#endif /* INC_PID_H_ */
