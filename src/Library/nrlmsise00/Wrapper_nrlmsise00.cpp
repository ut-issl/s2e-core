
/* ------------------------------------------------------------------- */
/* ------------------------------ INCLUDES --------------------------- */
/* ------------------------------------------------------------------- */

extern "C" {
#include <nrlmsise-00.h> /* header for nrlmsise-00.h */
}
#include <stdlib.h> /* for malloc/free */

#include <Environment/Global/PhysicalConstants.hpp>
#include <Library/math/Constant.hpp>
#include <algorithm>
#include <cctype>
#include <cmath> /* maths functions */
#include <numeric>

#include "Wrapper_nrlmsise00.h" /* header for nrlmsise-00.h */

using namespace std;

/* ------------------------------------------------------------------- */
/* ------------------------------ DEFINES ---------------------------- */
/* ------------------------------------------------------------------- */

static double decyear_monthly;

int LeapYear(int year) { return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0); }

void ConvertDaysToMonthDay(int days, int is_leap_year, int* month_day) {
  int days_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  if (is_leap_year) {
    days_month[1] = 29;
  }

  if (days <= days_month[0]) {
    month_day[0] = 1;
    month_day[1] = days;
  } else if (days > days_month[0] && days <= accumulate(days_month, days_month + 2, 0)) {
    month_day[0] = 2;
    month_day[1] = days - days_month[0];
  } else if (days > accumulate(days_month, days_month + 2, 0) && days <= accumulate(days_month, days_month + 3, 0)) {
    month_day[0] = 3;
    month_day[1] = days - accumulate(days_month, days_month + 2, 0);
  } else if (days > accumulate(days_month, days_month + 3, 0) && days <= accumulate(days_month, days_month + 4, 0)) {
    month_day[0] = 4;
    month_day[1] = days - accumulate(days_month, days_month + 3, 0);
  } else if (days > accumulate(days_month, days_month + 4, 0) && days <= accumulate(days_month, days_month + 5, 0)) {
    month_day[0] = 5;
    month_day[1] = days - accumulate(days_month, days_month + 4, 0);
  } else if (days > accumulate(days_month, days_month + 5, 0) && days <= accumulate(days_month, days_month + 6, 0)) {
    month_day[0] = 6;
    month_day[1] = days - accumulate(days_month, days_month + 5, 0);
  } else if (days > accumulate(days_month, days_month + 6, 0) && days <= accumulate(days_month, days_month + 7, 0)) {
    month_day[0] = 7;
    month_day[1] = days - accumulate(days_month, days_month + 6, 0);
  } else if (days > accumulate(days_month, days_month + 7, 0) && days <= accumulate(days_month, days_month + 8, 0)) {
    month_day[0] = 8;
    month_day[1] = days - accumulate(days_month, days_month + 7, 0);
  } else if (days > accumulate(days_month, days_month + 8, 0) && days <= accumulate(days_month, days_month + 9, 0)) {
    month_day[0] = 9;
    month_day[1] = days - accumulate(days_month, days_month + 8, 0);
  } else if (days > accumulate(days_month, days_month + 9, 0) && days <= accumulate(days_month, days_month + 10, 0)) {
    month_day[0] = 10;
    month_day[1] = days - accumulate(days_month, days_month + 9, 0);
  } else if (days > accumulate(days_month, days_month + 10, 0) && days <= accumulate(days_month, days_month + 11, 0)) {
    month_day[0] = 11;
    month_day[1] = days - accumulate(days_month, days_month + 10, 0);
  } else if (days > accumulate(days_month, days_month + 11, 0) && days <= accumulate(days_month, days_month + 12, 0)) {
    month_day[0] = 12;
    month_day[1] = days - accumulate(days_month, days_month + 11, 0);
  }
}

void ConvertDecyearToDate(double decyear, int* date) {
  // year
  int year = (int)(decyear);
  int is_leap_year = LeapYear(year);
  int days_per_year = is_leap_year ? 366 : 365;
  double reminder = decyear - year;

  // month day
  double days_d = reminder * days_per_year;
  int days = (int)days_d;
  if (days == 0) {
    days = 1;
  }
  reminder = days_d - (int)days_d;

  // hours
  double hours_d = reminder * 24;
  int hours = (int)hours_d;
  reminder = hours_d - (int)hours_d;

  // minutes
  double minutes_d = reminder * 60;
  int minutes = (int)minutes_d;
  reminder = minutes_d - (int)minutes_d;

  // second
  double seconds_d = reminder * 60;
  int seconds = (int)seconds_d;
  reminder = seconds_d - (int)seconds_d;

  int month_day[2];
  ConvertDaysToMonthDay(days, is_leap_year, month_day);

  date[0] = year;
  date[1] = month_day[0];
  date[2] = month_day[1];
  date[3] = hours;
  date[4] = minutes;
  date[5] = seconds;
}

double ConvertDateToDecyear(int year, int month, int day) {
  int is_leap_year = LeapYear(year);
  int days_per_year = is_leap_year ? 366 : 365;

  int days_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (is_leap_year) {
    days_month[1] = 29;
  }

  double days = (double)accumulate(days_month, days_month + month - 1, 0) + (double)day;

  return (double)year + days / (double)days_per_year;
}

int ConvertMonthStrToMonthNum(string month_str) {
  if (month_str == "Jan") {
    return 1;
  } else if (month_str == "Feb") {
    return 2;
  } else if (month_str == "Mar") {
    return 3;
  } else if (month_str == "Apr") {
    return 4;
  } else if (month_str == "May") {
    return 5;
  } else if (month_str == "Jun") {
    return 6;
  } else if (month_str == "Jul") {
    return 7;
  } else if (month_str == "Aug") {
    return 8;
  } else if (month_str == "Sep") {
    return 9;
  } else if (month_str == "Oct") {
    return 10;
  } else if (month_str == "Nov") {
    return 11;
  } else if (month_str == "Dec") {
    return 12;
  } else {
    return 0;
  }
}

/* ------------------------------------------------------------------- */
/* --------------------------CalcNRLMSISE00--------------------------- */
/* ------------------------------------------------------------------- */
/* GTD7 Wrapper */
double CalcNRLMSISE00(double decyear, double latrad, double lonrad, double alt, const vector<nrlmsise_table>& table, bool is_manual_param,
                      double manual_f107, double manual_f107a, double manual_ap) {
  struct nrlmsise_output output;
  struct nrlmsise_input input;
  struct nrlmsise_flags flags;
  struct ap_array aph;

  size_t i;
  int date[6];
  int idx = 0;

  /* input values */
  for (i = 0; i < 24; i++) {
    flags.switches[i] = 1;
  }

  ConvertDecyearToDate(decyear, date);

  input.doy = (decyear - (int)decyear) * 365.25;
  input.year = 0; /* without effect */
  input.sec = date[3] * 60.0 * 60.0 + date[4] * 60.0 + date[5];
  input.alt = alt / 1000.0;
  input.g_lat = latrad * libra::rad_to_deg;
  input.g_long = lonrad * libra::rad_to_deg;
  input.lst = input.sec / 3600.0 + lonrad * libra::rad_to_deg / 15.0;

  if (is_manual_param) {
    input.f107 = manual_f107;
    input.f107A = manual_f107a;
    input.ap = manual_ap;
  } else {
    // f10.7 and ap from table
    // テーブルサイズが0なら，0を返して終了
    if (table.size() == 0) {
      return 0.0;
    }

    // search table index
    for (i = 0; i < table.size(); i++) {
      if (decyear < decyear_monthly) {
        // 年月日が一致
        if ((date[0] == table[i].year) && (date[1] == table[i].month) && (date[2] == table[i].day)) {
          idx = i;
          break;
        }
      } else {
        // 年月が一致
        if ((date[0] == table[i].year) && (date[1] == table[i].month)) {
          idx = i;
          break;
        }
      }
    }

    input.f107A = table[idx].Ctr81_adj;
    input.f107 = table[idx].F107_adj;
    input.ap = table[idx].Ap_avg;
  }

  for (i = 0; i < 7; i++) {
    aph.a[i] = input.ap;
  }
  input.ap_a = &aph;

  gtd7(&input, &flags, &output);
  return output.d[5];
}

/* ------------------------------------------------------------------- */
/* -----------------------ReadSpaceWeatherTable----------------------- */
/* ------------------------------------------------------------------- */
/* URL for SpaceWeather.txt */
/* ftp://ftp.agi.com/pub/DynamicEarthData/SpaceWeather-v1.2.txt */

int GetSpaceWeatherTable_(double decyear, double endsec, const string& filename, vector<nrlmsise_table>& table) {
  ifstream ifs(filename);

  if (!ifs.is_open()) {
    cerr << "File open error (SpaceWether.txt)" << endl;
    return 0;
  }

  double decyear_ini = decyear;
  double decyear_end = decyear + endsec / 86400.0 / 365.0;
  int date_ini[6];
  int date_end[6];

  ConvertDecyearToDate(decyear_ini, date_ini);
  ConvertDecyearToDate(decyear_end, date_end);

  if (date_ini[0] < 2015 || date_ini[0] > 2043 || date_end[0] > 2043) {
    cerr << "Year must be between 2015 and 2043 for NRLMSISE00 atmosphere model" << endl;
  }

  string line;
  while (getline(ifs, line)) {
    nrlmsise_table line_data;

    string year_str = line.substr(0, 4);  // first 4 char
    if (!std::all_of(year_str.cbegin(), year_str.cend(), ::isdigit) || year_str.empty()) {
      if (year_str == "UPDA") {
        int year_updated = atoi(line.substr(8, 4).c_str());
        string month_updated_str = line.substr(13, 3).c_str();
        int month_updated = ConvertMonthStrToMonthNum(month_updated_str);
        int day_updated = atoi(line.substr(17, 2).c_str());

        // テーブルの更新日時を取得
        double decyear_updated = ConvertDateToDecyear(year_updated, month_updated, day_updated);

        // 更新日時から1カ月半以降は一日毎ではなく一月毎にデータが更新されるので，その日時をdecyearで計算しておく
        int days_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        decyear_monthly = decyear_updated + (days_month[month_updated] + 14) / 365.0;
      }
      continue;
    }

    int year = atoi(year_str.c_str());
    int month = atoi(line.substr(5, 2).c_str());
    int day = atoi(line.substr(8, 2).c_str());
    double decyear_line = ConvertDateToDecyear(year, month, day);
    // 少なくとも1カ月分はデータを確保するため，シミュレーション開始時刻の一ヶ月前からテーブル値を確保する
    double decyear_ini_ymd = ConvertDateToDecyear(date_ini[0], date_ini[1], date_ini[2]) - 31.0 / 365.0;  // 一ヶ月引く
    double decyear_end_ymd = ConvertDateToDecyear(date_end[0], date_end[1], date_end[2]);

    if (decyear_line < decyear_ini_ymd || decyear_line > decyear_end_ymd) continue;

    // Read table data
    line_data.year = year;
    line_data.month = month;
    line_data.day = day;
    line_data.Ap_avg = atof(line.substr(80, 3).c_str());
    line_data.F107_adj = atof(line.substr(93, 5).c_str());
    line_data.Ctr81_adj = atof(line.substr(101, 5).c_str());
    line_data.Lst81_adj = atof(line.substr(107, 5).c_str());
    line_data.F107_obs = atof(line.substr(113, 5).c_str());
    line_data.Ctr81_obs = atof(line.substr(119, 5).c_str());
    line_data.Lst81_obs = atof(line.substr(125, 5).c_str());

    table.push_back(line_data);
  }

  return table.size();
}
