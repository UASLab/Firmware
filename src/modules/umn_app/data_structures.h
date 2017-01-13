/*! 
 *
 * Derived from this file:
 *    https://github.com/UASLab/OpenFlight/blob/master/FlightCode/globaldefs.h
 *\file globaldefs.h
 *	\brief Global definitions
 *
 *	\details This file is used to define macros and structures used in the program
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: globaldefs_new_signals.h 854 2012-07-10 13:32:52Z joh07594 $
 */
#pragma once

/// IMU Data Structure
struct imu {
	float wx;	///< [rad/sec], body X axis angular rate (roll)
	float wy;	///< [rad/sec], body Y axis angular rate (pitch)
	float wz;	///< [rad/sec], body Z axis angular rate (yaw)
	float ax;	///< [m/sec^2], body X axis acceleration
	float ay;	///< [m/sec^2], body Y axis acceleration
	float az;	///< [m/sec^2], body Z axis acceleration
	float hx;	///< [Gauss], body X axis magnetic field
	float hy;	///< [Gauss], body Y axis magnetic field
	float hz;	///< [Gauss], body Z axis magnetic field
	double time; ///< [sec], timestamp of IMU data
};

/// GPS Data Structure
struct gps {
	float lat;	///< [deg], Geodetic latitude
	float lon;	///< [deg], Geodetic longitude
	float alt;	///< [m], altitude relative to WGS84
	float ve;	///< [m/sec], East velocity
	float vn;	///< [m/sec], North velocity
	float vd;	///< [m/sec], Down velocity
    float sig_N; ///< [m], Position error standard deviation in the North direction
    float sig_E; ///< [m], Position error standard deviation in the East direction
    float sig_D; ///< [m], Position error standard deviation in the Down direction
    float sig_vn; ///< [m/sec], Velocity error standard deviation in the North direction
    float sig_ve; ///< [m/sec], Velocity error standard deviation in the East direction
    float sig_vd; ///< [m/sec], Velocity error standard deviation in the Down direction
	float GPS_TOW;	///< [sec], GPS Time Of Week
	float courseOverGround;///< [rad], course over the ground, relative to true North
	float speedOverGround;	///< [rad], speed over the ground
	double time;	///< [sec], timestamp of GPS data
	unsigned short satVisible; ///< Number satellites used in the position solution
	unsigned short navValid;///< flag indicating whether the solution is valid, 0 = valid
	unsigned short GPS_week;///< GPS week since current epoch.
};

/// Air Data Structure
struct airdata {
	float h;		///< [m], barometric altitude above ground level (AGL)
	float ias;     ///< [m/sec], indicated airspeed
	float h_filt;	///< [m], filtered altitude
	float ias_filt;	///< [m/s], filtered airspeed
	float Ps;		///< [KPa], static pressure
	float Pd;		///< [KPa], dynamic pressure
	float aoa;		///< [rad], angle of attack from 5-hole Pitot probe
	float aos;		///< [rad], angle of sideslip from 5-hole Pitot probe
	float l_alpha; ///< [rad], angle of attack, from left vane
	float r_alpha;	///< [rad], angle of attack, from right vane
	float l_beta;	///< [rad], angle of sideslip, from left vane
	float r_beta;	///< [rad], angle of sideslip, from right vane
	float Pd_aoa;  ///< [KPa], dynamic pressure for aoa, from 5-hole Pitot probe
	float Pd_aos;	///< [KPa], dynamic pressure for aos, from 5-hole Pitot probe
	unsigned short status;	///< status bitfield for air data sensors.
};

/// Control surface deflections
struct surface {
	float dthr_pos;	///< [0-1], measured throttle position
	float de_pos;		///< [rad], measured elevator position, +TED
	float dr_pos; 		///< [rad], measured rudder position, +TEL
	float da_l_pos;	///< [rad], measured left aileron position, +TED
	float da_r_pos;	///< [rad], measured right aileron position, +TED
	float df_l_pos;	///< [rad], measured left flap position, +TED
	float df_r_pos;	///< [rad], measured right flap position, +TED
};


/// Control Data structure
struct control {
	float dthr;		///< [0-1], throttle command
	float de;			///< [rad], elevator command, +TED
	float dr; 			///< [rad], rudder command, +TEL
	float da_l;		///< [rad], left aileron command, +TED
	float da_r;		///< [rad], right aileron command, +TED
	float df_l;		///< [rad], left flap command, +TED
	float df_r;		///< [rad], right flap command, +TED
	float phi_cmd;		///< [rad], Euler roll angle command
	float theta_cmd;	///< [rad], Euler pitch angle command
	float psi_cmd;		///< [rad], Euler yaw angle command
	float p_cmd;		///< [rad/sec], body axis roll rate command
	float q_cmd;		///< [rad/sec], body axis pitch rate command
	float r_cmd;		///< [rad/sec], body axis yaw rate command
	float ias_cmd;		///< [m/sec], airspeed command
	float h_cmd;		///< [m], altitude command
	float gndtrk_cmd;	///< [rad], ground track angle command, relative to true north
	float aoa_cmd;		///< [rad], angle of attack command
	float aos_cmd;		///< [rad], angle of sideslip command
	float gamma_cmd;	///< [rad], flight path angle command
	float signal_0;     ///< user defined dummy variable
	float signal_1;     ///< user defined dummy variable
	float signal_2;     ///< user defined dummy variable
	float signal_3;     ///< user defined dummy variable
	float signal_4;     ///< user defined dummy variable
	float signal_5;     ///< user defined dummy variable
	float signal_6;     ///< user defined dummy variable
	float signal_7;     ///< user defined dummy variable
	float signal_8;     ///< user defined dummy variable
	float signal_9;     ///< user defined dummy variable
};

/// Navigation Filter Data Structure
struct nav {
	float lat;		///< [rad], geodetic latitude estimate
	float lon;		///< [rad], geodetic longitude estimate
	float alt;		///< [m], altitude relative to WGS84 estimate
	float vn;		///< [m/sec], north velocity estimate
	float ve;		///< [m/sec], east velocity estimate
	float vd;		///< [m/sec], down velocity estimate
	float phi;		///< [rad], Euler roll angle estimate
	float the;		///< [rad], Euler pitch angle estimate
	float psi;		///< [rad], Euler yaw angle estimate
	float quat[4];	///< Quaternions estimate
	float ab[3];	///< [m/sec^2], accelerometer bias estimate
	float gb[3];	///< [rad/sec], rate gyro bias estimate
	float asf[3];	///< [m/sec^2], accelerometer scale factor estimate
	float gsf[3];	///< [rad/sec], rate gyro scale factor estimate
	float Pp[3];	///< [rad], covariance estimate for position
	float Pv[3];	///< [rad], covariance estimate for velocity
	float Pa[3];	///< [rad], covariance estimate for angles
	float Pab[3];	///< [rad], covariance estimate for accelerometer bias
	float Pgb[3];	///< [rad], covariance estimate for rate gyro bias
	float Pasf[3];	///< [rad], covariance estimate for accelerometer scale factor
	float Pgsf[3];	///< [rad], covariance estimate for rate gyro scale factor
	double time;			///< [sec], timestamp of NAV filter
	float wn;			///< [m/s], estimated wind speed in the north direction
	float we;			///< [m/s], estimated wind speed in the east direction
	float wd;			///< [m/s], estimated wind speed in the down direction
	float signal_0;     ///< user defined dummy variable
	float signal_1;     ///< user defined dummy variable
	float signal_2;     ///< user defined dummy variable
	float signal_3;     ///< user defined dummy variable
	float signal_4;     ///< user defined dummy variable
	float signal_5;     ///< user defined dummy variable
	float signal_6;     ///< user defined dummy variable
	float signal_7;     ///< user defined dummy variable
	float signal_8;     ///< user defined dummy variable
	float signal_9;     ///< user defined dummy variable
};

/// Combined sensor data structure
struct sensordata {
	struct imu *imuData_ptr; 			///< pointer to imu data structure
	struct gps *gpsData_ptr;			///< pointer to gps data structure
	struct airdata *adData_ptr;			///< pointer to airdata data structure
};

