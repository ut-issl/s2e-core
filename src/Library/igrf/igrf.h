#ifndef __igrf_H__
#define __igrf_H__

extern double testglobal[3];

void set_file_path(const char *fname);
void field(double are, double aflat, double ara, int maxoda);
void tcoef(double *agh, double *aght, double atzero, int kexta, double aext[3]);
void tyear(double ayear);
void mfldg(double alat, double alon, double ahi, double *ax, double *ay, double *az, double *af);
void mfldc(double athe, double alon, double ar, double *ax, double *ay, double *az, double *af);
void gcomp(double *axg, double *ayg, double *azg);
void gigrf(int gen, double year);
void igrfc(double fido, double fkeido, double hght, double *tf);
void igrfresult(double *mag, double *theta);
void igrfm(double fm[6]);
void sigrf(double year);
void sdgrf(double year);
void spgrf(double year);
void igrfelement(double *mag, double *thetarad);
int TransMagaxisToECI(const double *mag, double *pos, double lonrad, double thetarad, double gmst);
void IgrfCalc(double decyear, double latrad, double lonrad, double alt, double side, double *mag);

#endif  //__igrf_H__