/*
 * KalmanFilter.c
 *
 *  Created on: Jan 28, 2022
 *      Author: fame
 */
#include <Library/KalmanFilter.h>

void KalmanFilter_initialise(KalmanFilter *dev, double x1,double x2,double p11,double p12,double p21,double p22, double R, double Q){
	/* Parameter */
	dev->R = R;
	dev->Q = Q;

	/* KF "memory" */
	dev->x1 = x1;
	dev->x2 = x2;
	dev->p11 = p11;
	dev->p12 = p12;
	dev->p21 = p21;
	dev->p22 = p22;
}

void KalmanFilter_Update(KalmanFilter *dev,double theta_k) {
	double X1 = (dev->x1);
	double X2 = (dev->x2);
	double P11 = (dev->p11);
	double P12 = (dev->p12);
	double P21 = (dev->p21);
	double P22 = (dev->p22);
	double Q = (dev->Q);
	double R = (dev->R);
	double dt_pow2 = DT * DT;
	double dt_pow3 = DT * DT * DT;
	double dt_pow4 = DT * DT * DT * DT;
	dev->x1 = X1 + X2*DT - ((X1 - theta_k + X2*DT)*(P11 + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT)))/(P11 + R + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT));
	dev->x2  = X2 - (((Q*dt_pow3)/2 + P22*DT + P21)*(X1 - theta_k + X2*DT))/(P11 + R + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT));
	dev->p11  = -((P11 + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT))/(P11 + R + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT)) - 1)*(P11 + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT));
	dev->p12 = -((P11 + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT))/(P11 + R + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT)) - 1)*((Q*dt_pow3)/2 + P22*DT + P12);
	dev->p21 = P21 + P22*DT + (Q*dt_pow3)/2 - (((Q*dt_pow3)/2 + P22*DT + P21)*(P11 + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT)))/(P11 + R + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT));
	dev->p22 = P22 + Q*dt_pow2 - (((Q*dt_pow3)/2 + P22*DT + P12)*((Q*dt_pow3)/2 + P22*DT + P21))/(P11 + R + P21*DT + (Q*dt_pow4)/4 + DT*(P12 + P22*DT));
}

