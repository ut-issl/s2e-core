/**********************/
/***     igrf.c     ***/
/****************************************************************************/
/* Usage from FORTRAN :     ( All variables are in single precision )       */
/*      First specify the Generation Number with the Year of Calc.          */
/*          by  CALL gigrf(NGEN, YEAR),                                     */
/*        or specify the Model Type (IGRF/DGRF/PGRF) with the Year of Calc. */
/*          by  CALL sigrf(YEAR), CALL sdgrf(YEAR) or CALL spgrf(YEAR) .    */
/*      Then, CALL igrfc(FI, FK, H, F) gives TotalForce (F) of that model   */
/*          at the point of Lat.=FI, Long.=FK, Alt.=H                       */
/*      If other components are desired, CALL igrfm(FM) .                   */
/*          Here FM is an array with 6 elements, which correspond to        */
/*              North(X), East(Y), Downward(Z), Horizontal(H) components,   */
/*              Inclination(I) and Declination(D).                          */
/*  Unit Convention:  Lat.(FI), Long.(FK), Inc.(I), Dec.(D) are in degrees, */
/*                    Mag.Fields(F,X,Y,Z,H) in nT, and Alt.(H) in meters.   */
/****************************************************************************/
/* Prototype definition for C :                                             */
/*--------------------------------------------------------------------------*/
/*   < Conventional Functions and their FORTRAN Interfaces >                */
/*--------------------------------------------------------------------------*/
/*  void sigrf(double year);         |  void sigrf_(float *year);           */
/*   / void sdgrf(double year);      |   / void sdgrf_(float *year);        */
/*   / void spgrf(double year);      |   / void spgrf_(float *year);        */
/*  void gigrf(int gen, double year);|  void gigrf_(int *gen, float *year); */
/*  void igrfc(double fi, double fk, |  void igrfc_(float *fi, float *fk,   */
/*             double h, double *f); |              float *h, float *f);    */
/*  void igrfm(double fm[6]);        |  void igrfm_(float fm[6]);           */
/*--------------------------------------------------------------------------*/
/*   < Substantial Calculation Functions >                                  */
/*--------------------------------------------------------------------------*/
/*  void field(double are, double aflat, double ara, int maxoda);           */
/*  void tcoef(double agh[MxOD+1][MxOD+1], double aght[MxOD+1][MxOD+1],     */
/*             double atzero, int kexta, double aext[3]);                   */
/*  void tyear(double ayear);                                               */
/*  void mfldg(double alat, double alon, double ahi,                        */
/*             double *ax, double *ay, double *az, double *af);             */
/*  void mfldc(double athe, double alon, double ar,                         */
/*             double *ax, double *ay, double *az, double *af);             */
/*  void gcomp(double *axg, double *ayg, double *azg);                      */
/****************************************************************************/

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <cstdlib>
#include <iostream>
using namespace std;

#include "../sgp4/sgp4ext.h"
#include "igrf.h"

double testglobal[3];

#pragma warning(disable : 4996)  // fopenなど回避
#pragma warning(disable : 4305)  // double->float回避

/*--------------------*/
/*   Basic Routines   */
/*--------------------*/

// TODO: Consider how to fix the following constant values in this library copied from outside

#define MxOD 19
#define URAD (180. / 3.14159265359)

#define PI 3.14159265358979323846
#define DEG2RAD 0.017453292519943295769236907684886  // PI/180
#define RAD2DEG (180 / PI)

static double ra, rpre, re, re2, re4, rp, rp2, rp4, tzero;
static int maxod, kg, kgc, kr, kth, kph, kext;
static double blat, blon, bhi, br, bthe;
static double rlat, slat, slat2, clat2;
static double r, the, phi, cth, sth, cph, sph;
static double x, y, z, f, ext0, ext1, ext2;
static double gh[MxOD + 1][MxOD + 1], ght[MxOD + 1][MxOD + 1], g[MxOD + 1][MxOD + 1];
static double rar[MxOD + 1], csp[MxOD + 1], snp[MxOD + 1], p[MxOD + 2][MxOD + 1];

static double vgh[MxOD + 1][MxOD + 1], vght[MxOD + 1][MxOD + 1];

// coeff file path
static char coeff_file[256];

static void fcalc(void);

void set_file_path(const char *fname) { strcpy(coeff_file, fname); }

static void fcalc(void) /* This is an internal function */
{
  double t, pn1m, tx, ty, tz;
  int n, m;
  if (kr != 0) {
    kr = 0;
    t = ra / r;
    rar[0] = t * t;
    for (n = 0; n < maxod; n++) rar[n + 1] = rar[n] * t;
  }
  if (kth != 0) {
    kth = 0;
    p[0][0] = 1.;
    p[1][0] = 0.;
    p[0][1] = cth;
    p[1][1] = sth;
    p[2][0] = -sth;
    p[2][1] = cth;
    for (n = 1; n < maxod; n++) {
      p[0][n + 1] = (p[0][n] * cth * (n + n + 1) - p[0][n - 1] * n) / (n + 1);
      p[n + 2][0] = (p[0][n + 1] * cth - p[0][n]) * (n + 1) / sth;
      for (m = 0; m <= n; m++) {
        pn1m = p[m][n + 1];
        p[m + 1][n + 1] = (p[m][n] * (n + m + 1) - pn1m * cth * (n - m + 1)) / sth;
        p[n + 2][m + 1] = pn1m * (n + m + 2) * (n - m + 1) - p[m + 1][n + 1] * cth * (m + 1) / sth;
      }
    }
  }
  if (kph != 0) {
    kph = 0;
    csp[0] = 1.;
    snp[0] = 0.;
    for (m = 0; m < maxod; m++) {
      csp[m + 1] = csp[m] * cph - snp[m] * sph;
      snp[m + 1] = snp[m] * cph + csp[m] * sph;
    }
  }
  x = 0.;
  y = 0.;
  z = 0.;
  for (n = 0; n < maxod; n++) {
    tx = g[0][n + 1] * p[n + 2][0];
    ty = 0.;
    tz = g[0][n + 1] * p[0][n + 1];
    for (m = 0; m <= n; m++) {
      tx += (g[m + 1][n + 1] * csp[m + 1] + g[n + 1][m] * snp[m + 1]) * p[n + 2][m + 1];
      ty += (g[m + 1][n + 1] * snp[m + 1] - g[n + 1][m] * csp[m + 1]) * p[m + 1][n + 1] * (m + 1);
      tz += (g[m + 1][n + 1] * csp[m + 1] + g[n + 1][m] * snp[m + 1]) * p[m + 1][n + 1];
    }
    x += rar[n + 1] * tx;
    y += rar[n + 1] * ty;
    z -= rar[n + 1] * tz * (n + 2);
  }
  y /= sth;
  if (kext != 0) {
    t = ext1 * cph + ext2 * sph;
    x -= (ext0 * cth + t * sth);
    y += (ext1 * sph - ext2 * cph);
    z += (ext0 * sth - t * cth);
  }
  f = sqrt(x * x + y * y + z * z);
}

void field(double are, double aflat, double ara, int maxoda) {
  ra = ara;
  maxod = maxoda;
  rpre = 1. - 1. / aflat;
  re = are;
  re2 = re * re;
  re4 = re2 * re2;
  rp = re * rpre;
  rp2 = rp * rp;
  rp4 = rp2 * rp2;
  kg = 2;
  kgc = 0;
  kph = 1;
}

void tcoef(double agh[MxOD + 1][MxOD + 1], double aght[MxOD + 1][MxOD + 1], double atzero, int kexta, double aext[3]) {
  int nn, mm;
  double fac;
  tzero = atzero;
  kext = kexta;
  gh[0][0] = 0.;
  ght[0][0] = 0.;
  for (nn = 1; nn <= maxod; nn++) {
    gh[0][nn] = agh[0][nn];
    ght[0][nn] = aght[0][nn];
    fac = sqrt(2.);
    for (mm = 1; mm <= nn; mm++) {
      fac /= sqrt((double)((nn + mm) * (nn - mm + 1)));
      gh[mm][nn] = agh[mm][nn] * fac;
      gh[nn][mm - 1] = agh[nn][mm - 1] * fac;
      ght[mm][nn] = aght[mm][nn] * fac;
      ght[nn][mm - 1] = aght[nn][mm - 1] * fac;
    }
  }
  if (kext == 0) {
    ext0 = 0.;
    ext1 = 0.;
    ext2 = 0.;
  } else {
    ext0 = aext[0];
    ext1 = aext[1];
    ext2 = aext[2];
  }
}

void tyear(double ayear) {
  double dyear;
  int nn, mm;
  dyear = ayear - tzero;
  for (nn = 0; nn <= maxod; nn++) {
    for (mm = 0; mm <= maxod; mm++) {
      g[mm][nn] = gh[mm][nn] + ght[mm][nn] * dyear;
    }
  }
}

void mfldg(double alat, double alon, double ahi, double *ax, double *ay, double *az, double *af) {
  double hi, rm2, rm, rrm;
  if ((kg != 1) || (blat != alat) || (bhi != ahi)) {
    kg = 1;
    kr = 1;
    kth = 1;
    blat = alat;
    bhi = ahi;
    rlat = alat / URAD;
    hi = ahi;
    slat = sin(rlat);
    slat2 = slat * slat;
    clat2 = 1. - slat2;
    rm2 = re2 * clat2 + rp2 * slat2;
    rm = sqrt(rm2);
    rrm = (re4 * clat2 + rp4 * slat2) / rm2;
    r = sqrt(rrm + 2. * hi * rm + hi * hi);
    cth = slat * (hi + rp2 / rm) / r;
    sth = sqrt(1. - cth * cth);
  }
  if (blon != alon) kph = 1;
  if (kph != 0) {
    blon = alon;
    phi = alon / URAD;
    cph = cos(phi);
    sph = sin(phi);
  }
  fcalc();
  *ax = x;
  *ay = y;
  *az = z;
  *af = f;
}

void mfldc(double athe, double alon, double ar, double *ax, double *ay, double *az, double *af) {
  if (kg == 0) {
    if (bthe != athe) kth = 1;
    if (br != ar) kr = 1;
  } else {
    kg = 0;
    kr = 1;
    kth = 1;
  }
  if (kr != 0) {
    br = ar;
    r = ar;
  }
  if (kth != 0) {
    bthe = athe;
    the = athe / URAD;
    cth = cos(the);
    sth = sin(the);
  }
  if (blon != alon) kph = 1;
  if (kph != 0) {
    blon = alon;
    phi = alon / URAD;
    cph = cos(phi);
    sph = sin(phi);
  }
  fcalc();
  *ax = x;
  *ay = y;
  *az = z;
  *af = f;
}

/*---------------------------*/
/*   Conventional Routines   */
/*---------------------------*/

#define MxGEN 13  // 12//11//10
#define RAD (180. / 3.14159265359)
#define MxMOD 19
#define MxELM ((MxMOD + 1) * (MxMOD + 1) - 1)
#define MxCOL 50
#define LLINE (MxCOL * 9 + 10)

// GIGRFの計算（G,D,P-GRFの区別なく計算）
// WGS84モデルで計算
void gigrf(int gen, double year) {
  int maxod, i, n, m, l, k, ncol, nlin;
  double y1, y2, yr1, yr2;
  double tzero, dmy[3], cb[MxELM], cv[MxELM];
  //係数表指定
  // char path[]="igrf10.coef";
  // char file[]="igrf10.coef";
  // char path[] = "igrf11.coef";
  // char file[] = "igrf11.coef";
  // char path[] = "src/Library/igrf/igrf11.coef";
  // char file[] = "src/Library/igrf/igrf11.coef";
  // char path[] = "../SatAttSim/src/Library/igrf/igrf12.coef";
  // char file[] = "../SatAttSim/src/Library/igrf/igrf12.coef";
  // char path[] = "../../SatAttSim/src/Library/igrf/igrf13.coef"; //from 2020
  // char file[] = "../../SatAttSim/src/Library/igrf/igrf13.coef"; //from 2020

  char file[256];

  char *pstr, *line, buf[LLINE];
  FILE *fp;
  if ((gen < 1) || (MxGEN < gen)) {
    fprintf(stderr, "gigrf: unknown  NGEN = %d\n", gen);
    exit(1);
  }
  strcpy(file, coeff_file);
  // strstr(const char *s1, const char *s2)=>文字列s1から文字列s2を検索
  // if ((pstr=strstr(file,"10")) == NULL)
  // if ((pstr = strstr(file, "11")) == NULL)
  // if ((pstr = strstr(file, "12")) == NULL)
  if ((pstr = strstr(file, "13")) == NULL) {
    fprintf(stderr, "gigrf: filename invalid\n");
    exit(1);
  }
  // sprintf(char *str, const char *format,  ．．．
  // )=printf関数と同様の変換を行った出力を、文字列strに格納 char *strncpy(char
  // *s1, const char *s2, size_t n);= *s1 に文字列 *s2 を先頭から n 文字コピー
  sprintf(buf, "%02d", gen);
  strncpy(pstr, buf, 2);
  if ((fp = fopen(file, "r")) == NULL) {
    fprintf(stderr, "gigrf: file not found\n");
    exit(1);
  }
  if (fgets(buf, LLINE, fp) == NULL) {
    fprintf(stderr, "gigrf: file empty\n");
    exit(1);
  }
  // int sscanf(const char *str, const char *format,  ．．．
  // );=strから書式formatにしたがって、scanf関数と同様の変換を行った入力を、指定されたアドレスに格納
  if (sscanf(buf, "%d%d%lf%lf", &maxod, &ncol, &y1, &y2) != 4) {
    fprintf(stderr, "gigrf: Line-1 format error\n");
    exit(1);
  }
  // for debug
  else {
    // fprintf(stderr, "coeff-matrix:%dby%d,valid period:%lfto%lf\n", maxod,
    // ncol, y1, y2);
  }
  if ((maxod < 8) || (maxod > MxMOD) || (ncol < 2) || (ncol > MxCOL)) {
    fprintf(stderr, "gigrf: Line-1 invalid\n");
    exit(1);
  }
  nlin = (maxod + 1) * (maxod + 1) - 1;
  if ((year < y1) || (year > y2)) fprintf(stderr, "gigrf: IGRF-%02d not defined for %9.3lf\n", gen, year);
  if (fgets(buf, LLINE, fp) == NULL) {
    fprintf(stderr, "gigrf: EOF before Line-2\n");
    exit(1);
  }
  line = &buf[1];
  if (sscanf(line, "%*c%*d%*d%lf%n", &yr2, &n) == EOF) {
    fprintf(stderr, "gigrf: Line-2 invalid\n");
    exit(1);
  }
  for (l = 2; l < ncol; l++) {
    line += n;
    yr1 = yr2;
    if (sscanf(line, "%lf%n", &yr2, &n) == EOF) {
      fprintf(stderr, "gigrf: Line-2 short\n");
      exit(1);
    }
    if (year < yr2) break;
  }
  for (i = 0; i < nlin; i++) {
    if (fgets(buf, LLINE, fp) == NULL) {
      fprintf(stderr, "gigrf: EOF before Line-%d\n", i + 3);
      exit(1);
    }
    line = &buf[0];
    if (sscanf(line, "%*c%*d%*d%n", &n) == EOF) {
      fprintf(stderr, "gigrf: Line-%d invalid\n", i + 3);
      exit(1);
    }
    for (m = 2; m < l; m++) {
      line += n;
      if (sscanf(line, "%*lf%n", &n) == EOF) {
        fprintf(stderr, "gigrf: Line-%d short\n", i + 3);
        exit(1);
      }
    }
    line += n;
    if (sscanf(line, "%lf%lf", &cb[i], &cv[i]) != 2) {
      fprintf(stderr, "gigrf: Line-%d short\n", i + 3);
      exit(1);
    }
  }
  if (l == ncol)
    tzero = yr2;
  else {
    tzero = yr1;
    yr2 -= yr1;
    for (i = 0; i < nlin; i++) cv[i] = (cv[i] - cb[i]) / yr2;
  }
  k = 0;
  for (i = 0, n = 1; n <= maxod; n++) {
    vgh[0][n] = cb[i];
    vght[0][n] = cv[i];
    i++;
    if ((cb[i] != 0.) || (cv[i] != 0.)) k = n;
    for (m = 1; m <= n; m++) {
      vgh[m][n] = cb[i];
      vght[m][n] = cv[i];
      i++;
      if ((cb[i] != 0.) || (cv[i] != 0.)) k = n;
      vgh[n][m - 1] = cb[i];
      vght[n][m - 1] = cv[i];
      i++;
      if ((cb[i] != 0.) || (cv[i] != 0.)) k = n;
    }
  }
  maxod = k;
  //ジオイドに関わる定数?? field(double are, double aflat, double ara, int
  // maxoda)
  field(6378.137, 298.25722, 6371.2, maxod);
  tcoef(vgh, vght, tzero, 0, dmy);
  tyear(year);
}

void igrfc(double fido, double fkeido, double hght, double *tf) {
  double fx, fy, fz;
  mfldg(fido, fkeido, hght / 1000., &fx, &fy, &fz, tf);
}

//地磁気要素（地心表現）をECI座標へ
int TransMagaxisToECI(const double *mag, double *pos, double lonrad, double thetarad, double gmst) {
  RotationY(mag, pos, 180 * DEG2RAD - thetarad);
  RotationZ(pos, pos, -lonrad);
  RotationZ(pos, pos, -gmst);

  return 0;
}

// IGRFの計算を実行するメインルーチン
// Output	:	mag[3]	ECI座標での磁界の値[nT]
void IgrfCalc(double decyear, double latrad, double lonrad, double alt, double side, double *mag) {
  static bool first_flg = true;

  double f;

  if (first_flg == true) {
    // gigrf(10,decyear);	//ファイル読み込み
    // gigrf(11, decyear);
    // gigrf(12, decyear);
    gigrf(13, decyear);
    first_flg = false;
  }
  tyear(decyear);  //実行年の設定
  igrfc(latrad * RAD2DEG, lonrad * RAD2DEG, alt,
        &f);  //実行位置の設定&Executeはこの中に含む

  mag[0] = x;  // x,y,zはigrf.cppグローバル変数
  mag[1] = y;
  mag[2] = z;

  // cout <<"mag_ned:["<< mag[0]<<","<<mag[1]<<","<<mag[2]<<"]\n";

  double thetarad = acos(cth);  //[0<=theta<=pi?]

  // cout << "Theta:"<< thetarad<<"\n";

  testglobal[0] = mag[0];
  testglobal[1] = mag[1];
  testglobal[2] = mag[2];

  TransMagaxisToECI(mag, mag, lonrad, thetarad, side);
}