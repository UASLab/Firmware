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
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file umn_algorithm.cpp
 *
 * @author HMokhtarzadeh <hamid@organicnavigation.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <cmath>
#include <drivers/drv_hrt.h>

#include <mathlib/mathlib.h> // Math and matrix library.

#include "data_structures.h" // Must come before `umn_algorithm.h`
#include "umn_algorithm.h"


using math::Matrix;
using math::Vector;

bool update(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	
	/* BEGIN YOUR ALGORITHM HERE
	 *   Step 1. Pull data from inputs
	 *   Step 2. Do your algorithm kung fu
	 *   Step 3. Pack output into `navData_ptr`
	 *	 Rinse and repeat...
	 *  
	 *   Some example code is provided below.  Feel free to remove.
	 */

	/* Access sensor data from `sensorData_ptr`. */
	// Example  AccelX:   sensorData_ptr->imuData_ptr->ax
	// Example    MagX:   sensorData_ptr->imuData_ptr->hx
	// Example GPS Lat:  sensorData_ptr->gpsData_ptr->lat
	// Example Baro-Alt: sensorData_ptr->adData_ptr->h
	float ax = sensorData_ptr->imuData_ptr->ax; // Accessing accelerometer data.


	/* Example Matrix Math */
	Matrix<2, 2> A;
	Matrix<2, 2> A_inv;
	Matrix<3, 3> B;
	A(0, 0) = 2;
	A(0, 1) = 0;
	A(1, 0) = 0;
	A(1, 1) = 3;
	A_inv = A.inversed();
	// (A * A_inv).print(); // (optional: print matrix to console)
	// (B*A).print(); //  Build-time error for incorrect matrix-math.

	/* Example Vector Math */
	math::Vector<3> a(1.0f, 0.0f, 0.4f);
	math::Vector<3> b(0.0f, 0.0f, 0.1f);
	math::Vector<4> c(0.0f, 0.0f, 0.0f, 1.0f);
	// a.print(); // (optional: print vector to console)
	// printf("%.3f\t", (double)(a*b)); // (optional: multiplication of two vectors is a float
									    //  hence we can print result with `printf` statement.)
	// (a*c); // Build-time error for incorrect vector-math.

	/* Example print message */
	PX4_INFO("Printed using `PX4_INFO`.  AccelX: %.3f", (double)ax);
	//printf("Printed using `printf`.  AccelX: %.3f \n", (double)ax);

	/* Push results into nav struct */
	navData_ptr->phi = 1;
	navData_ptr->the = -1;
	navData_ptr->psi = 0.25;
	// If valid update, return `true`.  Otherwise return `false`.
	return true;
}


