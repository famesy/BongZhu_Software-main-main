/*
 * KalmanFilter.h
 *
 *  Created on: Jan 28, 2022
 *      Author: fame
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_


#define DT 0.01

typedef struct {
	/* Parameter */
	double R;
	double Q;

	/* KF "memory" */
	double x1;
	double x2;
	double p11;
	double p12;
	double p21;
	double p22;
} KalmanFilter;

void KalmanFilter_initialise(KalmanFilter *dev, double x1,double x2,double p11,double p12,double p21,double p22, double R, double Q);
void KalmanFilter_Update(KalmanFilter *dev,double theta_k);
#endif /* INC_KALMANFILTER_H_ */
