/*
 *  Copyright (C) 2012, Laboratorio de Robotica Movel - ICMC/USP
 *  Rafael Luiz Klaser <rlklaser@gmail.com>
 *  http://lrm.icmc.usp.br
 *
 *  Apoio FAPESP: 2012/04555-4
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file WheelOdometryExtended.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 16, 2012
 *
 */


#include "lrm_odometry/WheelOdometryExtended.h"

#include <gsl/gsl_multimin.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_min.h>


double minimize_fuction (double x, void * params)
{
	struct st_odometry *odo = (struct st_odometry *)params;
	double minim = 0;

	//parameter to minimize function
	//odo.p.const_steer = x;
	odo->dist_f_r = odo->encWheelValue * odo->p.const_odom;
	odo->phi = (odo->encSteerValue - odo->p.steer_bar_zero)  * x; //odo->p->const_steer

	// - when the robot is heading straight, tan(phi)=0
	if(odo->phi!=0) {
		odo->a = tan(odo->phi);
		//rays
		odo->R = odo->L/odo->a; //ray based on steering (I->C->) - middle
		odo->R_r_l = odo->R - odo->b_2; //left triangle base
		odo->R_r_r = odo->R + odo->b_2; //right triangle base
		odo->R_f_l = sqrt(odo->L2 + odo->R_r_l*odo->R_r_l);
		odo->R_f_r = sqrt(odo->L2 + odo->R_r_r*odo->R_r_r);

		//calculate left/right wheel ratios
		odo->ratio = odo->R_f_l / odo->R_f_r;

		//ackerman wheels angles (just for printing)
		odo->phi_l = atan(odo->L/odo->R_r_l);
		odo->phi_r = atan(odo->L/odo->R_r_r);
	}
	odo->dist_f_l = odo->dist_f_r * odo->ratio;

	//middle front distance
	odo->dist_f = (odo->dist_f_r+odo->dist_f_l)/2;
	//rear distance (actual odometry distance)
	odo->k = sqrt(1+odo->a*odo->a);
	odo->dist_r = odo->dist_f / odo->k;
	odo->dist = odo->dist_r;

	//separetad rear distances (just for printing)
	odo->dist_r_l = odo->dist_f_l / odo->k;
	odo->dist_r_r = odo->dist_f_r / odo->k;

	//odo->dist_r_2 = (odo->dist_r_l + odo->dist_r_r) / 2; //just for testing (ok: equal)

	if(odo->p.use_imu) {
		odo->delta_theta = odo->yaw - odo->last_orientation;
	}
	else {
		if(odo->phi!=0) { //R == infinite
			odo->delta_theta = odo->dist / odo->R;
		}
		else {
			odo->delta_theta = 0;
		}
	}

	odo->delta_theta_2 = (odo->dist_r_r - odo->dist_r_l) / odo->b;

	//odo->new_phi = atan(odo->delta_theta * odo->L / odo->dist_r);

//	minim = sqrt((odo->delta_theta - odo->delta_theta_2) * (odo->delta_theta - odo->delta_theta_2));

//	ROS_INFO_STREAM(
//		" x:" << x <<
//		" dth:" << odo->delta_theta <<
//		" dt2:" << odo->delta_theta_2 <<
//		" z:" << minim
//	);

	//minimizing the distance between values == reached same value
	//return sqrt( (odo->delta_theta-odo->delta_theta_2) * (odo->delta_theta - odo->delta_theta_2)  );
	return minim;
}

int optimize (struct st_odometry *odo)
{
	int status;
	int iter = 0, max_iter = 100;
	const gsl_min_fminimizer_type *T;
	gsl_min_fminimizer *s;
	double m = odo->p.const_steer;
	double m_expected = odo->p.const_steer;
	double a = m - 0.00001;
	double b = m + 0.00001; //medir a folga
	gsl_function F;

	F.function = &minimize_fuction;
	F.params = odo;

	printf("%f %f %f\n", a, m, b);

	//T = gsl_min_fminimizer_goldensection;
	//T = gsl_min_fminimizer_brent
	T = gsl_min_fminimizer_quad_golden;
	s = gsl_min_fminimizer_alloc (T);

	try {
		gsl_min_fminimizer_set (s, &F, m, a, b);
	}
	catch(std::exception & e) {
		return -1;
	}

	printf ("using %s method\n", gsl_min_fminimizer_name (s));
	printf ("%5s [%9s, %9s] %9s %10s %9s\n", "iter", "lower", "upper", "min", "err", "err(est)");
	printf ("%5d [%.7f, %.7f] %.7f %+.7f %.7f\n", iter, a, b, m, m - m_expected, b - a);

	do
	{
		iter++;
		status = gsl_min_fminimizer_iterate (s);
		m = gsl_min_fminimizer_x_minimum (s);
		a = gsl_min_fminimizer_x_lower (s);
		b = gsl_min_fminimizer_x_upper (s);
		status = gsl_min_test_interval (a, b, 0.001, 0.0);

		if (status == GSL_SUCCESS)
			printf ("Converged:\n");

		printf ("%5d [%.7f, %.7f] " "%.7f %+.7f %.7f\n", iter, a, b, m, m - m_expected, b - a);
	}
	while (status == GSL_CONTINUE && iter < max_iter);

	gsl_min_fminimizer_free (s);

	return status;
}


void WheelOdometryExtended::calcOdometry()
{
	//position and orientation
	odo.delta_x = 0;
	odo.delta_y = 0;
	odo.delta_theta = 0;
	odo.delta_theta_2 = 0;

	odo.dist_f_r = 0;
	odo.R = 0;
	odo.R_f_l = 0;
	odo.R_f_r = 0;
	odo.ratio = 1;
	odo.phi_l = 0;
	odo.phi_r = 0;
	odo.a = 0;

	//optimize const_steer
	minimize_fuction(odo.p.const_steer, &odo);
	//optimize(&odo);
}

