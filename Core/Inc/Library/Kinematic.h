/*
 * kinematic.h
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#ifndef INC_KINEMATIC_H_
#define INC_KINEMATIC_H_

/*
 * Include
 */
#include "math.h"
#include "string.h"

/*
 * DEFINES
 */
#define PI 3.142857

/*
 * STRUCT
 */

/*
 * FUNCTIONS
 */
int IPK(double x, double y, double z, double pitch, double roll, double *config_arr);
void IVK(double q[5], double x_dot[5], double *m_dot);


#endif /* INC_KINEMATIC_H_ */
