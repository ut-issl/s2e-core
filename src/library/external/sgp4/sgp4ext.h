#ifndef _sgp4ext_
#define _sgp4ext_
/*     ----------------------------------------------------------------
 *
 *                                 sgp4ext.h
 *
 *    this file contains extra routines needed for the main test program for
 * sgp4. these routines are derived from the astro libraries.
 *
 *                            companion code for
 *               fundamentals of astrodynamics and applications
 *                                    2007
 *                              by david vallado
 *
 *       (w) 719-573-2600, email dvallado@agi.com
 *
 *    current :
 *              20 apr 07  david vallado
 *                           misc documentation updates
 *    changes :
 *              14 aug 06  david vallado
 *                           original baseline
 *       ---------------------------------------------------------------- */

#include <math.h>
#include <string.h>

// ------------------------- function declarations -------------------------

int RotationX(const double* bfr, double* aft, double theta);
int RotationY(const double* bfr, double* aft, double theta);
int RotationZ(const double* bfr, double* aft, double theta);
double AcTan(double sinx, double cosx);
double FMod2p(double x);
double sgn(double);

double mag(double[]);

void cross(double[], double[], double[]);

double dot(double[], double[]);

double CalcAngleTwoVectors_rad(double[], double[]);

void newtonnu(double ecc, double nu, double& e0, double& m);

void rv2coe(double[], double[], double, double&, double&, double&, double&, double&, double&, double&, double&, double&, double&, double&);

void jday(int, int, int, int, int, double, double&);

void days2mdhms(int, double, int&, int&, int&, int&, double&);

void invjday(double, int&, int&, int&, int&, int&, double&);
void JdToDecyear(double jd, double* decyear);

float juldayTEST(int, int, int);

#endif
