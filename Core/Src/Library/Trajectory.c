/*
 * Trajectory.c
 *
 *  Created on: Feb 23, 2022
 *      Author: fame
 */

#include <Library/Trajectory.h>

void QuinticTrajectory_initialise(QuinticTrajectory *Traj, double time_step) {
	Traj->is_end = 1;
	Traj->c0 = 0;
	Traj->c1 = 0;
	Traj->c2 = 0;
	Traj->c3 = 0;
	Traj->c4 = 0;
	Traj->c5 = 0;
	Traj->pos_out = 0;
	Traj->vel_out = 0;
	Traj->current_time = 0;
	Traj->time_step = time_step;
	Traj->time_max = 0;
}

void QuinticTrajectory_cal_and_set_coeff(QuinticTrajectory *Traj, double q0,
		double q1, double v0, double v1, double ac0, double ac1, double tf) {
	static const signed char iv[6] = { 1, 0, 0, 0, 0, 0 };
	static const signed char iv1[6] = { 0, 1, 0, 0, 0, 0 };
	static const signed char iv2[6] = { 0, 0, 2, 0, 0, 0 };
	double M[36];
	double a[6];
	double M_tmp;
	double s;
	double smax;
	int b_M_tmp;
	int b_a;
	int b_tmp;
	int i;
	int j;
	int jA;
	int jp1j;
	int k;
	int mmj_tmp;
	signed char ipiv[6];
	signed char i1;
	M[3] = 1.0;
	M[9] = tf;
	smax = tf * tf;
	M[15] = smax;
	s = pow(tf, 3.0);
	M[21] = s;
	M_tmp = pow(tf, 4.0);
	M[27] = M_tmp;
	M[33] = pow(tf, 5.0);
	M[4] = 0.0;
	M[10] = 1.0;
	M[16] = 2.0 * tf;
	M[22] = 3.0 * smax;
	M[28] = 4.0 * s;
	M[34] = 5.0 * M_tmp;
	M[5] = 0.0;
	M[11] = 0.0;
	M[17] = 2.0;
	M[23] = 6.0 * tf;
	M[29] = 12.0 * smax;
	M[35] = 20.0 * s;
	/*  */
	a[0] = q0;
	a[1] = v0;
	a[2] = ac0;
	a[3] = q1;
	a[4] = v1;
	a[5] = ac1;
	for (i = 0; i < 6; i++) {
		M[6 * i] = iv[i];
		M[6 * i + 1] = iv1[i];
		M[6 * i + 2] = iv2[i];
		ipiv[i] = (signed char) (i + 1);
	}
	for (j = 0; j < 5; j++) {
		mmj_tmp = 4 - j;
		b_tmp = j * 7;
		jp1j = b_tmp + 2;
		jA = 6 - j;
		b_a = 0;
		smax = fabs(M[b_tmp]);
		for (k = 2; k <= jA; k++) {
			s = fabs(M[(b_tmp + k) - 1]);
			if (s > smax) {
				b_a = k - 1;
				smax = s;
			}
		}
		if (M[b_tmp + b_a] != 0.0) {
			if (b_a != 0) {
				jA = j + b_a;
				ipiv[j] = (signed char) (jA + 1);
				for (k = 0; k < 6; k++) {
					b_a = j + k * 6;
					smax = M[b_a];
					b_M_tmp = jA + k * 6;
					M[b_a] = M[b_M_tmp];
					M[b_M_tmp] = smax;
				}
			}
			i = (b_tmp - j) + 6;
			for (b_a = jp1j; b_a <= i; b_a++) {
				M[b_a - 1] /= M[b_tmp];
			}
		}
		jA = b_tmp;
		for (b_a = 0; b_a <= mmj_tmp; b_a++) {
			smax = M[(b_tmp + b_a * 6) + 6];
			if (smax != 0.0) {
				i = jA + 8;
				b_M_tmp = (jA - j) + 12;
				for (jp1j = i; jp1j <= b_M_tmp; jp1j++) {
					M[jp1j - 1] += M[((b_tmp + jp1j) - jA) - 7] * -smax;
				}
			}
			jA += 6;
		}
		i1 = ipiv[j];
		if (i1 != j + 1) {
			smax = a[j];
			a[j] = a[i1 - 1];
			a[i1 - 1] = smax;
		}
	}
	for (k = 0; k < 6; k++) {
		jA = 6 * k;
		if (a[k] != 0.0) {
			i = k + 2;
			for (b_a = i; b_a < 7; b_a++) {
				a[b_a - 1] -= a[k] * M[(b_a + jA) - 1];
			}
		}
	}
	for (k = 5; k >= 0; k--) {
		jA = 6 * k;
		smax = a[k];
		if (smax != 0.0) {
			smax /= M[k + jA];
			a[k] = smax;
			for (b_a = 0; b_a < k; b_a++) {
				a[b_a] -= a[k] * M[b_a + jA];
			}
		}
	}
	Traj->is_end = 0;
	Traj->c0 = a[0];
	Traj->c1 = a[1];
	Traj->c2 = a[2];
	Traj->c3 = a[3];
	Traj->c4 = a[4];
	Traj->c5 = a[5];
	Traj->current_time = 0;
	Traj->time_max = tf;
}

void QuinticTrajectory_set_param(QuinticTrajectory *Traj, double c0, double c1,
		double c2, double c3, double c4, double c5, double time_max) {
	Traj->is_end = 0;
	Traj->c0 = c0;
	Traj->c1 = c1;
	Traj->c2 = c2;
	Traj->c3 = c3;
	Traj->c4 = c4;
	Traj->c5 = c5;
	Traj->current_time = 0;
	Traj->time_max = time_max;
}

void QuinticTrajectory_update(QuinticTrajectory *Traj) {
	if (!(Traj->is_end)) {
		if (Traj->current_time > Traj->time_max) {
			Traj->current_time = Traj->time_max;
			Traj->is_end = 1;
		}
		Traj->pos_out = Traj->c0 + (Traj->c1 * Traj->current_time)
				+ (Traj->c2 * pow(Traj->current_time, 2))
				+ (Traj->c3 * pow(Traj->current_time, 3))
				+ (Traj->c4 * pow(Traj->current_time, 4))
				+ (Traj->c5 * pow(Traj->current_time, 5));
		Traj->vel_out = Traj->c1 + (2 * Traj->c2 * Traj->current_time)
				+ (3 * Traj->c3 * pow(Traj->current_time, 2))
				+ (4 * Traj->c4 * pow(Traj->current_time, 3))
				+ (5 * Traj->c5 * pow(Traj->current_time, 4));
		Traj->current_time += Traj->time_step;
	}
}

void pathway(double Xstart, double Nstart, double Xstop, double Nstop,
		double qStart, double bSpeed, double trajectCoef[210]) {
	static const double dv[7] = { 1.0, 0.5, 0.5, 2.0, 0.5, 0.5, 1.0 };
	static const double dv1[5] = { 216.7107, 0.0, 221.8579, 0.0, 0.0 };
	static const signed char iv[6] = { 1, 0, 0, 0, 0, 0 };
	static const signed char iv1[6] = { 0, 1, 0, 0, 0, 0 };
	static const signed char iv2[6] = { 0, 0, 2, 0, 0, 0 };
	double viaPoint[40];
	double viaPointQ[40];
	double A[36];
	double timeCon[7];
	double c[6];
	double bSp;
	double b_r__tmp;
	double dis;
	double dx;
	double dy;
	double q3;
	double q3_max;
	double rSam;
	double r_;
	double r__tmp;
	double z_;
	int A_tmp;
	int a;
	int b_i;
	int b_tmp;
	int i;
	int j;
	int jA;
	int jp1j;
	int k;
	int mmj_tmp;
	int q_;
	char st[24];
	signed char ipiv[6];
	signed char i1;
	/* constant */
	/* totalTime */
	/*       */
	bSp = bSpeed * 2.0 * 3.1415926535897931;
	for (i = 0; i < 7; i++) {
		timeCon[i] = 0.0;
	}
	timeCon[0] = 1.0;
	for (i = 0; i < 6; i++) {
		timeCon[i + 1] = timeCon[i] + dv[i + 1];
	}
	/* via point and time to go there */
	dis = (Xstart - 4.5) * 50.0;
	rSam = (4.5 - Nstart) * 50.0;
	q3_max = sqrt(dis * dis + rSam * rSam);
	/* via point and time to go there */
	/* via point and time to go there */
	/* via point and time to go there */
	dy = (Xstop - 4.5) * 50.0;
	dx = (4.5 - Nstop) * 50.0;
	r_ = sqrt(dy * dy + dx * dx);
	/* via point and time to go there */
	/* via point and time to go there */
	dis = atan2(rSam, dis);
	rSam = (qStart + bSp) + dis;
	viaPoint[1] = q3_max * cos(rSam) + 450.0;
	viaPoint[9] = q3_max * sin(rSam);
	viaPoint[17] = 220.0;
	viaPoint[25] = 1.5707963267948966;
	viaPoint[33] = 0.0;
	rSam = (qStart + bSp * timeCon[1]) + dis;
	viaPoint[2] = q3_max * cos(rSam) + 450.0;
	viaPoint[10] = q3_max * sin(rSam);
	viaPoint[18] = 100.0;
	viaPoint[26] = 1.5707963267948966;
	viaPoint[34] = 0.0;
	rSam = (qStart + bSp * timeCon[2]) + dis;
	viaPoint[3] = q3_max * cos(rSam) + 450.0;
	viaPoint[11] = q3_max * sin(rSam);
	viaPoint[19] = 220.0;
	viaPoint[27] = 1.5707963267948966;
	viaPoint[35] = 0.0;
	dis = atan2(dx, dy);
	rSam = (qStart + bSp * timeCon[3]) + dis;
	viaPoint[4] = r_ * cos(rSam) + 450.0;
	viaPoint[12] = r_ * sin(rSam);
	viaPoint[20] = 220.0;
	viaPoint[28] = 1.5707963267948966;
	viaPoint[36] = 0.0;
	rSam = (qStart + bSp * timeCon[4]) + dis;
	viaPoint[5] = r_ * cos(rSam) + 450.0;
	viaPoint[13] = r_ * sin(rSam);
	viaPoint[21] = 100.0;
	viaPoint[29] = 1.5707963267948966;
	viaPoint[37] = 0.0;
	rSam = (qStart + bSp * timeCon[5]) + dis;
	viaPoint[6] = r_ * cos(rSam) + 450.0;
	viaPoint[14] = r_ * sin(rSam);
	viaPoint[22] = 220.0;
	viaPoint[30] = 1.5707963267948966;
	viaPoint[38] = 0.0;
	for (b_i = 0; b_i < 5; b_i++) {
		jA = b_i << 3;
		q3_max = dv1[b_i];
		viaPoint[jA] = q3_max;
		viaPoint[jA + 7] = q3_max;
	}
	/*      viaPoint */
	/*  IK */
	for (i = 0; i < 8; i++) {
		r__tmp = viaPoint[i + 8];
		b_r__tmp = -viaPoint[i + 24];
		q3_max = viaPoint[i];
		r_ = sqrt(q3_max * q3_max + r__tmp * r__tmp) - 146.0 * cos(b_r__tmp);
		z_ = (viaPoint[i + 16] - 236.0) - 146.0 * sin(b_r__tmp);
		dis = sqrt(r_ * r_ + z_ * z_);
		rSam = 0.0;
		bSp = 0.0;
		q3_max = 1.5707963267948966;
		q3 = 0.0;
		dy = 0.0;
		dx = 0.0;
		while (fabs(rSam - dis) > 0.01) {
			q3 = (q3_max + bSp) / 2.0;
			dy = -410.48751503547584 * cos(q3 + 0.04874185130993159)
					+ 352.27829907617075 * cos(q3 - 0.11379200714370806);
			dx = (410.48751503547584 * sin(q3 + 0.04874185130993159) + 60.0)
					+ 352.27829907617075 * sin(q3 - 0.11379200714370806);
			rSam = sqrt(dx * dx + dy * dy);
			if (rSam > dis) {
				q3_max = q3;
			} else {
				bSp = q3;
				/*          i += 1 */
			}
		}
		/*  # if i > 10:break */
		/*      b = atan2(dy , dx)*180/pi */
		/*      a = (pi / 2 - q3 + k1)*180/pi */
		q3_max = ((1.5707963267948966 - q3) + 0.11379200714370806)
				- atan2(dy, dx);
		rSam = atan(z_ / r_);
		bSp = (rSam + q3_max) - 2.4699864973360528;
		if ((bSp < -2.4699864973360528) || (bSp > 0.0)) {
			dis = round(bSp * 180.0 / 3.1415926535897931);
			if ((!(dis == 0.0)) && (dis)) {
				sprintf(&st[0], "%.16g", dis);
			}
		}
		dis = (3.1415926535897931 - (2.0 * q3 + -0.065050155833776463))
				- q3_max;
		/*      (pi / 4 - atan(l3_1 / l3_2)) */
		bSp += 0.78539816339744828;
		q3 -= 0.78539816339744828;
		viaPointQ[i] = atan2(r__tmp, viaPoint[i]);
		viaPointQ[i + 8] = bSp;
		viaPointQ[i + 16] = q3;
		viaPointQ[i + 24] = (((b_r__tmp - rSam) + dis) - 0.7366563120875167)
				+ 0.78539816339744828;
		viaPointQ[i + 32] = viaPoint[i + 32];
		/*      display(i) */
		/*      display([x,y]) */
		/*      [x,y,z] =
		 * FK(viaPointQ(i,1),viaPointQ(i,2),viaPointQ(i,3),viaPointQ(i,4)) */
	}
	/*  Trajectory */
	/* mm/s */
	/* mm/s^2 */
	/*  velocity that conected to each path */
	/*  acerelation that conected to each path */
	for (i = 0; i < 7; i++) {
		bSp = dv[i];
		q3_max = pow(bSp, 5.0);
		dy = bSp * bSp;
		dx = pow(bSp, 3.0);
		r_ = pow(bSp, 4.0);
		for (q_ = 0; q_ < 5; q_++) {
			/*          display(i) */
			/*          display([p_i,pf]) */
			A[3] = 1.0;
			A[9] = bSp;
			A[15] = dy;
			A[21] = dx;
			A[27] = r_;
			A[33] = q3_max;
			A[4] = 0.0;
			A[10] = 1.0;
			A[16] = 2.0 * bSp;
			A[22] = 3.0 * dy;
			A[28] = 4.0 * dx;
			A[34] = 5.0 * r_;
			A[5] = 0.0;
			A[11] = 0.0;
			A[17] = 2.0;
			A[23] = 6.0 * bSp;
			A[29] = 12.0 * dy;
			A[35] = 20.0 * dx;
			jA = i + (q_ << 3);
			c[0] = viaPointQ[jA];
			c[1] = 0.0;
			c[2] = 0.0;
			c[3] = viaPointQ[jA + 1];
			c[4] = 0.0;
			c[5] = 0.0;
			for (b_i = 0; b_i < 6; b_i++) {
				A[6 * b_i] = iv[b_i];
				A[6 * b_i + 1] = iv1[b_i];
				A[6 * b_i + 2] = iv2[b_i];
				ipiv[b_i] = (signed char) (b_i + 1);
			}
			for (j = 0; j < 5; j++) {
				mmj_tmp = 4 - j;
				b_tmp = j * 7;
				jp1j = b_tmp + 2;
				jA = 6 - j;
				a = 0;
				dis = fabs(A[b_tmp]);
				for (k = 2; k <= jA; k++) {
					rSam = fabs(A[(b_tmp + k) - 1]);
					if (rSam > dis) {
						a = k - 1;
						dis = rSam;
					}
				}
				if (A[b_tmp + a] != 0.0) {
					if (a != 0) {
						jA = j + a;
						ipiv[j] = (signed char) (jA + 1);
						for (k = 0; k < 6; k++) {
							a = j + k * 6;
							dis = A[a];
							A_tmp = jA + k * 6;
							A[a] = A[A_tmp];
							A[A_tmp] = dis;
						}
					}
					b_i = (b_tmp - j) + 6;
					for (a = jp1j; a <= b_i; a++) {
						A[a - 1] /= A[b_tmp];
					}
				}
				jA = b_tmp;
				for (a = 0; a <= mmj_tmp; a++) {
					dis = A[(b_tmp + a * 6) + 6];
					if (dis != 0.0) {
						b_i = jA + 8;
						A_tmp = (jA - j) + 12;
						for (jp1j = b_i; jp1j <= A_tmp; jp1j++) {
							A[jp1j - 1] += A[((b_tmp + jp1j) - jA) - 7] * -dis;
						}
					}
					jA += 6;
				}
				i1 = ipiv[j];
				if (i1 != j + 1) {
					dis = c[j];
					c[j] = c[i1 - 1];
					c[i1 - 1] = dis;
				}
			}
			for (k = 0; k < 6; k++) {
				jA = 6 * k;
				if (c[k] != 0.0) {
					b_i = k + 2;
					for (a = b_i; a < 7; a++) {
						c[a - 1] -= c[k] * A[(a + jA) - 1];
					}
				}
			}
			for (k = 5; k >= 0; k--) {
				jA = 6 * k;
				dis = c[k];
				if (dis != 0.0) {
					dis /= A[k + jA];
					c[k] = dis;
					for (a = 0; a < k; a++) {
						c[a] -= c[k] * A[a + jA];
					}
				}
			}
			for (b_i = 0; b_i < 6; b_i++) {
				trajectCoef[(q_ + 5 * b_i) + 30 * i] = c[b_i];
			}
		}
	}
}
