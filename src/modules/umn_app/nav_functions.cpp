/*! \file nav_functions.cpp 
*	\brief Auxiliary functions for nav filter
 *
 *	\details
 *     Module:          Navfuncs.c
 *     Modified:        Inchara Lakshminarayan (Quaternion functions, Port to px4 Matrix library) 
			Adhika Lie (revamp all functions)
 * 			Gokhan Inalhan (remaining)
 *                      Demoz Gebre (first three functions)
 *                      Jung Soon Jang
 *
 *     Description:     navfunc.c contains the listing for all the
 *                      real-time inertial navigation software.
 *
 *		Note: all the functions here do not create memory without
 *			  clearing it.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id$
 */

#include <matrix/math.hpp>
#include "nav_functions.h"
using namespace matrix;

/*=================================================================*/
Matrix<double, 3,3> eul2dcm(Vector<double, 3> euler)
{
	/* Function:     MATRIX eul2dcm(MATRIX euler, MATRIX dcm)
	 * ----------------------------------------------------------------
	 * This function creates the direction cosine matrix (DCM) that
	 * transforms a vector from navigation frame to the body frame given 
	 * a set of Euler Angle in the form of (phi theta psi) for a 3-2-1 
	 * rotation sequence
	 */
  double cPHI,sPHI,cTHE,sTHE,cPSI,sPSI;
  Matrix<double, 3,3> dcm;

  cPHI = cos(euler(0)); sPHI = sin(euler(0));
  cTHE = cos(euler(1)); sTHE = sin(euler(1));
  cPSI = cos(euler(2)); sPSI = sin(euler(2));

  dcm(0,0) = cTHE*cPSI; 				dcm(0,1) = cTHE*sPSI; 					dcm(0,2) = -sTHE;
  dcm(1,0) = sPHI*sTHE*cPSI-cPHI*sPSI;	dcm(1,1) = sPHI*sTHE*sPSI+cPHI*cPSI;	dcm(1,2) = sPHI*cTHE;
  dcm(2,0) = cPHI*sTHE*cPSI+sPHI*sPSI;	dcm(2,1) = cPHI*sTHE*sPSI-sPHI*cPSI;	dcm(2,2) = cPHI*cTHE;

  return(dcm);
}

/*=================================================================*/
Vector<double, 3> dcm2eul(Matrix<double, 3,3> dcm) 
{
	/*
	* Function:     MATRIX dcm2eul(MATRIX euler, MATRIX dcm)
	*-----------------------------------------------------------------
	* Convert *any* DCM into its Euler Angle equivalent. For navigatin, 
	* use DCM from NED to Body-fixed as input to get the conventional 
	* euler angles.
	* The output argument 'euler' is a vector containing the
	* the three euler angles in radians given in (phi; theta; psi) format.
	* Modified: Adhika Lie, 09/13/2011.
	*/

	Vector<double, 3> euler;

	euler(0) = atan2(dcm(1,2),dcm(2,2));
	euler(1) = -asin(dcm(0,2));
	euler(2) = atan2(dcm(0,1),dcm(0,0));
	
	return euler;
}

/*=================================================================*/
Matrix<double, 3,3>  create_R(Vector<double, 3> e)
{
	/* This function is used to create the transformation matrix to get
	 * phi_dot, the_dot and psi_dot from given pqr (body rate).
	 */
	double ph, th;
	Matrix<double, 3,3> R;

	ph = e(0); 
	th = e(1); 
	
	R(0,0) = 1.0;
	R(0,1) = sin(ph)*tan(th);
	R(0,2) = cos(ph)*tan(th);
	
	R(1,0) = 0.0;
	R(1,1) = cos(ph);
	R(1,2) = -sin(ph);
	
	R(2,0) = 0.0;
	R(2,1)= sin(ph)/cos(th);
	R(2,2) = cos(ph)/cos(th);
	
	return R;
}

/*=================================================================*/
Vector<double, 3> llarate(Vector<double, 3> V, Vector<double, 3> lla) 
{
	/* This function calculates the rate of change of latitude, longitude,
	 * and altitude.
	 * Using WGS-84.
	 */
	double lat, h, Rew, Rns, denom;
	Vector<double, 3> lla_dot;

	lat = lla(0); h = lla(2);
	
	denom = (1.0 - (ECC2 * sin(lat) * sin(lat)));
	denom = sqrt(denom*denom);

	Rew = EARTH_RADIUS / sqrt(denom);
	Rns = EARTH_RADIUS*(1-ECC2) / denom*sqrt(denom);
	
	lla_dot(0) = V(0)/(Rns + h);
	lla_dot(1) = V(1)/((Rew + h)*cos(lat));
	lla_dot(2) = -V(2);
	
	return lla_dot;
}

/*=================================================================*/
Vector<double, 3> navrate(Vector<double, 3> V, Vector<double, 3> lla) 
{
	/* This function calculates the angular velocity of the NED frame, 
	 * also known as the navigation rate.
	 * Using WGS-84.
	 */
	double lat, h, Rew, Rns, denom;
	Vector<double, 3> nr;

	lat = lla(0); 
	h = lla(2);
	
	denom = (1.0 - (ECC2 * sin(lat) * sin(lat)));
	denom = sqrt(denom*denom);

	Rew = EARTH_RADIUS / sqrt(denom);
	Rns = EARTH_RADIUS*(1-ECC2) / denom*sqrt(denom);
	
	nr(0) = V(1)/(Rew + h);
	nr(1) = -V(0)/(Rns + h);
	nr(2) = -V(1)*tan(lat)/(Rew + h);
	
	return nr;
}

/*=================================================================*/
Vector<double, 3>  ecef2lla(Vector<double, 3> ecef)
{
	/* This function calculates the Latitude, Longitude and Altitude of a
	 * point located on earth given the ECEF Coordinate.
	 * Reference: Jekeli, C.,"Inertial Navigation Systems With Geodetic 
	         Applications", Walter de Gruyter, New York, 2001, pp. 24
	*/
	double x, y, z;
	double lat, lon, alt, p, err, denom, Rew;
	Vector<double, 3> lla;

	x = ecef(0); y = ecef(1); z = ecef(2);
	lon = atan2(y,x);
	
	p = sqrt(x*x+y*y);

	lat = atan2(z,p*(1-ECC2));
	
	err = 1.0;
	while (fabs(err)>1e-14){
		denom = (1.0 - (ECC2 * sin(lat) * sin(lat)));
		denom = sqrt(denom*denom);

		Rew = EARTH_RADIUS / sqrt(denom);
		
		alt = p/cos(lat) - Rew;
		
		err = atan2(z*(1+ECC2*Rew*sin(lat)/z),p) - lat;
		lat = lat + err;
	}
	
	lla(0) = lat;
	lla(1) = lon;
	lla(2) = alt;
	
	return lla;
}

/*=================================================================*/
Vector<double, 3>  lla2ecef(Vector<double, 3>  lla)
{  
	/* This function calculates the ECEF Coordinate given the Latitude,
	 * Longitude and Altitude.
	 */
	
	double Rew, alt, denom;
	double sinlat, coslat, coslon, sinlon;
	Vector<double, 3> ecef;

	sinlat = sin(lla(0));
	coslat = cos(lla(0));
	coslon = cos(lla(1));
	sinlon = sin(lla(1));
	alt = lla(2);

	denom = (1.0 - (ECC2 * sinlat * sinlat));
	denom = sqrt(denom*denom);

	Rew = EARTH_RADIUS / sqrt(denom);
  
	ecef(0) = (Rew + alt) * coslat * coslon;
	ecef(1) = (Rew + alt) * coslat * sinlon;
	ecef(2)= (Rew * (1.0 - ECC2) + alt) * sinlat;
	
	return ecef;
}

/*=================================================================*/
Vector<double, 3> ecef2ned(Vector<double, 3> ecef, Vector<double, 3> pos_ref)
{
	/* This function converts a vector in ecef to ned coordinate centered
	 * at ecef_ref.
	 */
	//MATRIX lla_ref = mat_creat(3,3,ZERO_MATRIX);
	double lat, lon;
	Vector<double, 3> ned;
	
	//lla_ref = ecef2lla(ecef_ref, lla_ref);
	//lat = lla_ref(0)(0);
	//lon = lla_ref(1)(0);

	lat = pos_ref(0);
	lon = pos_ref(1);
	
	ned(2)=-cos(lat)*cos(lon)*ecef(0)-cos(lat)*sin(lon)*ecef(1)-sin(lat)*ecef(2);
	ned(1)=-sin(lon)*ecef(0) + cos(lon)*ecef(1);
	ned(0)=-sin(lat)*cos(lon)*ecef(0)-sin(lat)*sin(lon)*ecef(1)+cos(lat)*ecef(2);
	
	//mat_free(lla_ref);
	
	return ned;
}

/*=================================================================*/
Matrix<double, 3,3> sk(Vector<double, 3>  w)
{
	/* This function gives a skew symmetric matrix from a given vector w
	 */
	Matrix<double, 3,3> C;

	C = zeros<double, 3, 3>();
	C(0,1) = -w(2);	C(0,2) = w(1);
	C(1,0) = w(2);		C(1,2) = -w(0);
	C(2,0) = -w(1);	C(2,1) = w(0);	
	
	return C;
}

/*=================================================================*/
Matrix<double, 3,3> ortho(Matrix<double, 3,3> C)
{
	/*
	This function orthogonalizes a DCM by method presented in the paper
	Bar-Itzhack: "Orthogonalization Techniques of DCM" (1969, IEEE)
	Input:
		C: DCM, 3x3
	Output:
		C_ortho: Orthogonalized DCM, 3x3
	Programmer:    Adhika Lie
	Created:    	 May 10, 2011
	Last Modified: May 10, 2011
	*/
	Matrix<double, 3,3> C_ortho;

	Vector<double,3> e;
	Vector<double,3> w1;
	Vector<double,3> w1_p;
	Vector<double,3> w1_n;
	Vector<double,3> w2;
	Vector<double,3> w2_p;
	Vector<double,3> w2_n;
	Vector<double,3> w3;
	Vector<double,3> w3_p;
	Vector<double,3> w3_n;
	
	e.setOne();
	double mag_w1, mag_w2, mag_w3;
	
	w1(0) = C(0,0); 
	w1(1) = C(1,0);
	w1(2) = C(2,0);
	mag_w1 = norm(w1);
	mag_w1 = 1.0/mag_w1;
	w1 = w1*mag_w1;
	
	w2(0) = C(0,1); 
	w2(1) = C(1,1);
	w2(2) = C(2,1);
	mag_w2 = norm(w2);
	mag_w2 = 1.0/mag_w2;
	w2 = w2*mag_w2;
	
	w3(0) = C(0,2); 
	w3(1) = C(1,2);
	w3(2) = C(2,2);
	mag_w3 = norm(w3);
	mag_w3 = 1.0/mag_w3;
	w3 = w3*mag_w3;
	
	while (norm(e) > 1e-15){
		w1_p = cross(w2,w3);
		w2_p = cross(w3,w1);
		w3_p = cross(w1,w2);
		
		w1_n = w1+w1_p;
		w1_n = w1_n*0.5;
		w1 = w1_n/norm(w1_n);
		
		w2_n = w2+w2_p;
		w2_n = w2_n*0.5;
		w2 = w2_n/norm(w2_n);
		
		w3_n = w3+w3_p;
		w3_n = w3_n*0.5;
		w3 = w3_n/norm(w3_n);
		
		w1_p = cross(w2,w3);
		w2_p = cross(w3,w1);
		w3_p = cross(w1,w2);
		
		w1_n = w1-w1_p;
		e(0) = norm(w1_n);
		w2_n = w2-w2_p;
		e(1) = norm(w2_n);
		w3_n = w3-w3_p;
		e(2) = norm(w3_n);
	}
	
	C_ortho(0,0) = w1(0); C_ortho(0,1) = w2(0);	C_ortho(0,2) = w3(0);
	C_ortho(1,0) = w1(1); C_ortho(1,1) = w2(1);	C_ortho(1,2) = w3(1);
	C_ortho(2,0) = w1(2); C_ortho(2,1) = w2(2);	C_ortho(2,2) = w3(2);
	
	return C_ortho;
}

double norm(Vector<double, 3> a) 
{
	int i;
	double mag = 0.0;
	
	for (i=0;i<3;i++) mag += a(i)*a(i);
	
	return sqrt(mag);
}

Vector<double, 3> cross (Vector<double, 3> a, Vector<double, 3> b)
{
	// c = a x b;
	Vector<double, 3> c;

	c(0) = a(1)*b(2) - a(2)*b(1);
	c(1) = a(2)*b(0) - a(0)*b(2);
	c(2) = a(0)*b(1) - a(1)*b(0);
	return c;
}


/*=====================================================================*/
/*======================= QUATERNION FUNCTIONS ========================*/
/*=====================================================================*/
Vector<double, 4> qmult(Vector<double, 4> p, Vector<double, 4> q)
{
	/* Quaternion Multiplication: r = p x q
	 */
	int i;
	Vector<double, 4> r;
	for(i=0;i<3;i++) r(i) = 0.0;
	
	r(0) = p(0)*q(0) - (p(1)*q(1) + p(2)*q(2) + p(3)*q(3));
	r(1) = p(0)*q(1) + q(0)*p(1) + p(2)*q(3) - p(3)*q(2);
	r(2) = p(0)*q(2) + q(0)*p(2) + p(3)*q(1) - p(1)*q(3);
	r(3) = p(0)*q(3) + q(0)*p(3) + p(1)*q(2) - p(2)*q(1);

	return r;
}

Vector<double, 3> quat2eul(Vector<double, 4> q) {
	// Quaternion to Euler Angle
	double m11, m12, m13, m23, m33;
	Vector<double, 3> euler;

	m11 = 2*q(0)*q(0) +2*q(1)*q(1) -1;
	m12 = 2*q(1)*q(2) + 2*q(0)*q(3);
	m13 = 2*q(1)*q(3) - 2*q(0)*q(2);
	m23 = 2*q(2)*q(3) + 2*q(0)*q(1);
	m33 = 2*q(0)*q(0) + 2*q(3)*q(3) - 1;
	
	euler(2) = atan2(m12,m11);
	euler(1) = asin(-m13);
	euler(0) = atan2(m23,m33);
	
	return euler;
}

Vector<double, 4> eul2quat(Vector<double, 3> euler) {
	double phi,the,psi;
	phi = euler(0)/2.0;
	the = euler(1)/2.0;
	psi = euler(2)/2.0;
	Vector<double, 4> q;

	q(0) = cos(psi)*cos(the)*cos(phi) + sin(psi)*sin(the)*sin(phi);  
	q(1) = cos(psi)*cos(the)*sin(phi) - sin(psi)*sin(the)*cos(phi);
	q(2) = cos(psi)*sin(the)*cos(phi) + sin(psi)*cos(the)*sin(phi);  
	q(3) = sin(psi)*cos(the)*cos(phi) - cos(psi)*sin(the)*sin(phi);

	return q;
}

Matrix<double, 3,3> quat2dcm(Vector<double, 4> q) {
	// Quaternion to C_N2B
	Matrix<double, 3,3> C_N2B;

	C_N2B(0,0) = 2*q(0)*q(0) - 1 + 2*q(1)*q(1);
	C_N2B(1,1) = 2*q(0)*q(0) - 1 + 2*q(2)*q(2);
	C_N2B(2,2) = 2*q(0)*q(0) - 1 + 2*q(3)*q(3);
	
	C_N2B(0,1) = 2*q(1)*q(2) + 2*q(0)*q(3);
	C_N2B(0,2) = 2*q(1)*q(3) - 2*q(0)*q(2);
	
	C_N2B(1,0) = 2*q(1)*q(2) - 2*q(0)*q(3);
	C_N2B(1,2) = 2*q(2)*q(3) + 2*q(0)*q(1);
	
	C_N2B(2,0) = 2*q(1)*q(3) + 2*q(0)*q(2);
	C_N2B(2,1) = 2*q(2)*q(3) - 2*q(0)*q(1);
	
	return C_N2B;
}

