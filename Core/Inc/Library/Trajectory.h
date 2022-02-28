/*
 * kinematic.h
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

/*
 * Include
 */
#include "math.h"
#include "stdint.h"
#include "string.h"

/*
 * DEFINES
 */

/*
 * STRUCT
 */

typedef struct {
	/* memory */
	uint8_t is_end; // 0 = still_work , 1 = finished
	double time_step;
	double time_max;
	double c0;
	double c1;
	double c2;
	double c3;
	double c4;
	double c5;
	double current_time;
	double pos_out;
	double vel_out;
} QuinticTrajectory;

/*
 * FUNCTIONS
 */

void QuinticTrajectory_initialise(QuinticTrajectory *Traj, double time_step);
void QuinticTrajectory_cal_and_set_coeff(QuinticTrajectory *Traj, double q0, double q1,
		double v0, double v1, double ac0, double ac1, double tf);
void QuinticTrajectory_set_param(QuinticTrajectory *Traj, double c0, double c1, double c2, double c3, double c4, double c5,double time_max);
void QuinticTrajectory_update(QuinticTrajectory *Traj);

void pathway(double Xstart, double Nstart, double Xstop, double Nstop,
		double qStart, double bSpeed, double trajectCoef[210]);

#endif /* INC_TRAJECTORY_H_ */
