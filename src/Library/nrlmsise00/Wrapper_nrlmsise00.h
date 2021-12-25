#pragma once

#ifndef __wrapper_nrlmsise00_H__
#define __wrapper_nrlmsise00_H__


#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>


struct nrlmsise_table {
  int year;
  int month;
  int day;
  double Ap_avg;
  double F107_adj;
  double Ctr81_adj;
  double Lst81_adj;
  double F107_obs;
  double Ctr81_obs;
  double Lst81_obs;
};


/* CalcNRLMSISE00  */
/* GTD7 Wrapper */
double CalcNRLMSISE00(double decyear, double latrad, double lonrad, double alt, const std::vector<nrlmsise_table>& table);

/* GetSpaceWeatherTable_*/
int GetSpaceWeatherTable_(double decyear, double endsec, const std::string& filename, std::vector<nrlmsise_table>& table);

/* ------------------------------------------------------------------- */
/* ----------------------- COMPILATION TWEAKS ------------------------ */
/* ------------------------------------------------------------------- */

/* "inlining" of functions */
/*   Some compilers (e.g. gcc) allow the inlining of functions into the
 *   calling routine. This means a lot of overhead can be removed, and
 *   the execution of the program runs much faster. However, the filesize
 *   and thus the loading time is increased.
 */
#ifdef INLINE
#define __inline_double static inline double
#else
#define __inline_double double
#endif

#endif //__wrapper_nrlmsise00_H__
