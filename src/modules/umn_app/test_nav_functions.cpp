/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.`	
 *
 ****************************************************************************/

/* About: This is a test file to check all the functions in nav_functions.cpp
	5/16/2017 Inchara Lakshminarayan      */

#include <px4_config.h>
#include <px4_defines.h>
#include <cmath>
#include <drivers/drv_hrt.h>

#include <matrix/math.hpp>

// Include headers for uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/umn_output.h>

#include "umn_algorithm.h"
#include "nav_functions.h"

using namespace matrix;

#define D2R		0.017453292519943	///< [rad] degrees to radians */
#define R2D		57.295779513082323	///< [deg] radians to degrees */
#define PI      	3.14159265358979    ///< pi */
#define PI2     	6.28318530717958	///< pi*2 */
#define half_pi		1.57079632679490	///< pi/2 */

bool update(uint64_t time_usec, struct sensor_combined_s *sensorCombined_ptr, struct airspeed_s *airSpeed_ptr, struct vehicle_gps_position_s *gps_ptr, struct vehicle_land_detected_s *landDetected_ptr, struct umn_output_s *uout_ptr){
	
/* 	Matrix/vector declarations 	*/	
	Matrix<double, 3,3> C;
	Vector<double, 3> euler;
	Matrix<double, 3,3> R;
	Vector<double, 3> e;
	Vector<double, 3> V;
	Vector<double, 3> lla;
	Vector<double, 3> lla_dot;
	Vector<double, 3> nr;
	Vector<double, 3> ecef;
	Vector<double, 3> pos_ref;
	Vector<double, 3> ned;
	Vector<double, 3> w;
	Vector<double, 3> a;
	Vector<double, 3> b;
	Vector<double, 3> c;
	Vector<double, 4> q;
	Vector<double, 4> p;
	Vector<double, 4> r;

//--------------------------------------------------------------------------------//
	printf("\nCheck eul2dcm: \n");
	euler(0) = PI;
	euler(1) = 0.0;
	euler(2) = 0.0;
	printf("Euler angles %f %f %f \n", euler(0) , euler(1), euler(2));
	printf("DCM\n");
	C = eul2dcm(euler);
	C.print();

//--------------------------------------------------------------------------------//
	printf("\nCheck dcm2eul: \n");
	C.setZero();
	C(0,0) = 1.0;
	C(1,1) = 1.0;
	C(2,2) = -1.0;
	euler = dcm2eul(C);
	printf("DCM \n");
	C.print();
	printf("Euler angles %f %f %f \n", euler(0) , euler(1), euler(2));

//--------------------------------------------------------------------------------//
	printf("\nCheck create_R \n");
	e(0) = PI;
	e(1) = 0.0;
	e(2) = 0.0;
	printf("Body rate %f %f %f \n", e(0) , e(1), e(2));
	printf("Transformation matrix\n");
	R = create_R(e);
	R.print();

//--------------------------------------------------------------------------------//
	printf("\nCheck llarate\n");
	V.setZero();
	V(1) = EARTH_RADIUS;
	lla(0) = 0.0;
	lla(1) = 0.0;
	lla(2) = 0.0;
	printf("V %f %f %f \n", V(0) , V(1), V(2));
	printf("lla %f %f %f \n", lla(0) , lla(1), lla(2));
	lla_dot = llarate(V,lla);
	printf("lla_dot %f %f %f \n", lla_dot(0) , lla_dot(1), lla_dot(2));

//--------------------------------------------------------------------------------//
	printf("\nCheck navrate\n");
	V.setZero();
	V(1) = EARTH_RADIUS;
	lla(0) = 0.0;
	lla(1) = 0.0;
	lla(2) = 0.0;
	printf("V %f %f %f \n", V(0) , V(1), V(2));
	printf("lla %f %f %f \n", lla(0) , lla(1), lla(2));
	nr = navrate(V,lla);
	printf("nr %f %f %f \n", nr(0) , nr(1), nr(2));

//--------------------------------------------------------------------------------//
	printf("\nCheck ecef2lla\n");
	ecef(0) = EARTH_RADIUS;
	ecef(1) = 0.0;
	ecef(2) = 1.0;
	printf("Ecef %f %f %f \n", ecef(0) , ecef(1), ecef(2));
	lla = ecef2lla(ecef);
	printf("lla %f %f %f \n", lla(0) , lla(1), lla(2));

//--------------------------------------------------------------------------------//
	printf("\nCheck lla2ecef\n");
	lla(0) = 0.0;
	lla(1) = 0.0;
	lla(2) = 0.0;
	printf("lla %f %f %f \n", lla(0) , lla(1), lla(2));
	ecef = lla2ecef(lla);
	printf("Ecef %f %f %f \n", ecef(0) , ecef(1), ecef(2));
	
//--------------------------------------------------------------------------------//
	printf("\nCheck ecef2ned \n");
	pos_ref.setZero();
	ecef(0) = EARTH_RADIUS;
	ecef(1) = 0.0;
	ecef(2) = 0.0;
	printf("pos_ref %f %f %f \n", pos_ref(0) , pos_ref(1), pos_ref(2));
	printf("ecef %f %f %f \n", ecef(0) , ecef(1), ecef(2));
	ned = ecef2ned(ecef,pos_ref);
	printf("ned %f %f %f \n", ned(0) , ned(1), ned(2));	

//--------------------------------------------------------------------------------//
	printf("\nCheck sk\n");
	w(0) = 1.0;
	w(1) = 2.0;
	w(2) = 3.0;	
	printf("w %f %f %f \n", w(0) , w(1), w(2));
	C = sk(w);
	printf("C\n");
	C.print();

//--------------------------------------------------------------------------------//
	printf("\nCheck ortho \n");
	C.setIdentity();
	C(0,0) += 0.5;
	C(2,2) -= 0.2;
	printf("Before \n");
	C.print();
	C = ortho(C);
	printf("After \n");
	C.print();

//--------------------------------------------------------------------------------//
	printf("Check norm\n");
	a(0) = 1;
	a(1) = 1;
	a(2) = 1.4142;
	printf("a %f %f %f \n", a(0) , a(1), a(2));
	printf("Norm %f \n",norm(a));
	
//--------------------------------------------------------------------------------//
	printf("\nCheck cross\n");
	a.setZero();
	a(0) = 1.0;
	b.setZero();
	b(1) = 1.0;
	c = cross(a,b);
	printf("a %f %f %f \n", a(0) , a(1), a(2));
	printf("b %f %f %f \n", b(0) , b(1), b(2));
	printf("c %f %f %f \n", c(0) , c(1), c(2));

//--------------------------------------------------------------------------------//
	printf("\nCheck qmult\n");
	p.setZero();
	p(0) = 1.0;
	q.setZero();
	q(1) = 1.0;
	printf("p %f %f %f %f\n", p(0) , p(1), p(2), p(3));
	printf("q %f %f %f %f\n", q(0) , q(1), q(2), q(3));
	r = qmult(p,q);
	printf("r %f %f %f %f\n", r(0) , r(1), r(2), r(3));

//--------------------------------------------------------------------------------//
	printf("\nCheck quat2eul\n");
	q.setZero();
	q(1) = 1.0;
	printf("q %f %f %f %f\n", q(0) , q(1), q(2), q(3));
	euler = quat2eul(q);
	printf("Euler angles %f %f %f \n", euler(0) , euler(1), euler(2));

//--------------------------------------------------------------------------------//
	printf("\nCheck eul2quat: \n");
	euler(0) = PI;
	euler(1) = 0.0;
	euler(2) = 0.0;
	printf("Euler angles %f %f %f \n", euler(0) , euler(1), euler(2));
	q = eul2quat(euler);
	printf("q %f %f %f %f\n", q(0) , q(1), q(2), q(3));	

//--------------------------------------------------------------------------------//
	printf("\nCheck quat2dcm\n");
	q.setZero();
	q(0) = 1.0;
	printf("q %f %f %f %f\n", q(0) , q(1), q(2), q(3));
	C = quat2dcm(q);
	printf("DCM\n");
	C.print(); 


	// If valid update, return `true`.  Otherwise return `false`.	
	return true;
}

