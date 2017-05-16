/*! \file nav_functions.h
 *	\brief Auxiliary functions for nav filter header file
 *
 *	\details
 *     Module:          navfunc.h
 *     Modified:        Inchara Lakshminarayan (Quaternion functions, Port to px4 Matrix library) 
			Gokhan Inalhan (remaining) 
 *                      Demoz Gebre (first three functions)
 *                      Adhika Lie
 *                      Jung Soon Jang
 *     Description:     navfunc.h contains all the variable, 
 *                      constants and function prototypes that are 
 *                      used with the inertial navigation software.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id$
 */

//#include "matrix.h"
#include <matrix/math.hpp>
using namespace matrix;

#ifndef NAV_FUNCTIONS_H_
#define NAV_FUNCTIONS_H_


/*     Define Constants   */

#define EARTH_RATE   0.00007292115   /* rotation rate of earth (rad/sec) */
#define EARTH_RADIUS 6378137.0       /* earth semi-major axis radius (m) */
#define ECCENTRICITY 0.0818191908426 /* major eccentricity of earth ellipsoid */
#define ECC2		 0.00669437999014 /* major eccentricity squared */
#define FLATTENING   0.0033528106650 /* flattening of the ellipsoid */
#define GRAVITY_0    9.7803730       /* zeroth coefficient for gravity model */
#define GRAVITY_1    0.0052891       /* first coefficient for the gravity model*/ 
#define GRAVITY_2    0.0000059       /* second coefficient for the gravity model*/
#define GRAVITY_NOM  9.81            /* nominal gravity */ 
#define SCHULER2     1.533421593170545E-06 /* Sculer Frequency (rad/sec) Squared */
//#define R2D          57.29577951308232     /* radians to degrees conversion factor */
//#define D2R          0.01745329251994      /* degrees to radians conversion factor */  
#define FT2M         0.3048                /* feet to meters conversion factor */
#define KTS2ms       0.5144                /* Knots to meters/sec conversion factor*/
//#define PI           3.14159265358979      /* pi */
#define MAG_DEC      0.270944862           /*magnetic declination of Stanford (rad): 15.15 degrees */
#define MM2M         0.001                 /*mm to m*/

/*---------------     Define Structures and Enumerated Types -------------*/
typedef enum {OFF, ON} toggle;


Matrix<double, 3,3> eul2dcm(Vector<double, 3> euler);

Vector<double, 3> dcm2eul(Matrix<double, 3,3> dcm);

Matrix<double, 3,3>  create_R(Vector<double, 3> e);

Vector<double, 3> llarate(Vector<double, 3> V, Vector<double, 3> lla); 

Vector<double, 3> navrate(Vector<double, 3> V, Vector<double, 3> lla);

Vector<double, 3>  ecef2lla(Vector<double, 3> ecef);

Vector<double, 3>  lla2ecef(Vector<double, 3>  lla);

Vector<double, 3> ecef2ned(Vector<double, 3> ecef, Vector<double, 3> pos_ref);

Matrix<double, 3,3> sk(Vector<double, 3>  w);

Matrix<double, 3,3> ortho(Matrix<double, 3,3> C);

double norm(Vector<double, 3> a) ;

Vector<double, 3> cross (Vector<double, 3> a, Vector<double, 3> b);

Vector<double, 4> qmult(Vector<double, 4> p, Vector<double, 4> q);

Vector<double, 3> quat2eul(Vector<double, 4> q);

Vector<double, 4> eul2quat(Vector<double, 3> euler);

Matrix<double, 3,3> quat2dcm(Vector<double, 4> q);

#endif








