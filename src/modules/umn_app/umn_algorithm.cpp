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


using math::Matrix;
using math::Vector;

bool update(uint64_t time_usec, struct sensor_combined_s *sensorCombined_ptr, struct airspeed_s *airSpeed_ptr, struct vehicle_gps_position_s *gps_ptr, struct vehicle_land_detected_s *landDetected_ptr, struct umn_output_s *uout_ptr){
	
	/* BEGIN YOUR ALGORITHM HERE
	 *   Step 1. Pull data from inputs
	 *   Step 2. Do your algorithm kung fu
	 *   Step 3. Pack output into `uout_ptr`  (elements described here: ~/src/Firmware/msg/umn_output.msg)
	 *	 Rinse and repeat...
	 *  
	 *   Some example code is provided below.  Feel free to remove.
	 */

	/* Access sensor data from `sensorData_ptr`. */
	// Example  AccelX:   sensorCombined_ptr->accelerometer_m_s2[0]
	// Example    MagX:   sensorCombined_ptr->magnetometer_ga[0]
	// Example GPS Lat:  gps_ptr->lat  // Latitude in 1E-7 degrees
	// Example Baro-Alt: sensorCombined_ptr->baro_alt_meter
	// Example Indicated Airspeed:  airSpeed_ptr->indicated_airspeed_m_s
	float ax = sensorCombined_ptr->accelerometer_m_s2[0]; // Accessing accelerometer data.


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
	PX4_INFO("Printed using `PX4_INFO`. (%10d usec) AccelX: %.3f", time_usec, (double)ax);
	//printf("Printed using `printf`.  AccelX: %.3f \n", (double)ax);

	/* Push results into nav struct */
    uout_ptr->timestamp = time_usec; 

    uout_ptr->yaw_rad = 1.0;
    uout_ptr->roll_rad = -1.0;
    uout_ptr->pitch_rad = 0.5;

	// If valid update, return `true`.  Otherwise return `false`.
	return true;
}


