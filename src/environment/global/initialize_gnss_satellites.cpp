/**
 *@file initialize_gnss_satellites.cpp
 *@brief Initialize functions for GnssSatellites class
 */

#include "initialize_gnss_satellites.hpp"

#include <interface/initialize/initialize_file_access.hpp>

#include <iostream>
#include <string>

std::string return_dirctory_path(std::string sort) {
  std::string main_directory, sub_directory;

  if (sort.substr(0, 2) == "IG") {
    main_directory = "IGS/";
    if (sort.at(2) == 'S') {
      sub_directory = "igs/";
    } else if (sort.at(2) == 'R') {
      sub_directory = "igr/";
    } else if (sort.at(2) == 'U') {
      sub_directory = "igu/";
    }
  } else if (sort.substr(0, 2) == "ma") {
    main_directory = "JAXA/";
    sub_directory = "madoca/";
  } else {
    for (int i = 0; i < (int)sort.size(); ++i) {
      if (sort.at(i) == '_') {
        main_directory = sort.substr(0, i) + "/";
        if (sort.at(i + 1) == 'F') {
          sub_directory = "final/";
        } else if (sort.at(i + 1) == 'R') {
          sub_directory = "rapid/";
        } else if (sort.at(i + 1) == 'U') {
          sub_directory = "ultra_rapid/";
        } else {
          std::cout << "file_sort has something wrong" << std::endl;
          exit(1);
        }
        break;
      }
    }
  }

  return main_directory + sub_directory;
}

void get_raw_contents(std::string directory_path, std::string file_name, std::vector<std::string>& storage) {
  std::string all_file_path = directory_path + file_name;
  std::ifstream ifs(all_file_path);

  if (!ifs.is_open()) {
    std::cout << "in " << directory_path << "gnss file: " << file_name << " not found" << std::endl;
    exit(1);
  }
  std::string str;
  while (std::getline(ifs, str)) {
    storage.push_back(str);
  }
  ifs.close();
  if (storage.back() == "EOF") storage.pop_back();

  return;
}

void get_sp3_file_contents(std::string directory_path, std::string file_sort, std::string first, std::string last,
                           std::vector<std::vector<std::string>>& file_contents, UR_KINDS& ur_flag) {
  std::string all_directory_path = directory_path + return_dirctory_path(file_sort);
  ur_flag = UR_NOT_UR;

  if (first.substr(0, 3) == "COD") {
    std::string file_header = "COD0MGXFIN_";
    std::string file_footer = "0000_01D_05M_ORB.SP3";
    int year = stoi(first.substr(file_header.size(), 4));
    int year_last_day = 365 + (year % 4 == 0) - (year % 100 == 0) + (year % 400 == 0);
    int day = stoi(first.substr(file_header.size() + 4, 3));

    file_contents.clear();

    while (true) {
      if (day > year_last_day) {
        ++year;
        year_last_day = 365 + (year % 4 == 0) - (year % 100 == 0) + (year % 400 == 0);
        day = 1;
      }
      std::string s_day;
      if (day >= 100)
        s_day = std::to_string(day);
      else if (day >= 10)
        s_day = '0' + std::to_string(day);
      else
        s_day = "00" + std::to_string(day);
      std::string file_name = file_header + std::to_string(year) + s_day + file_footer;
      file_contents.push_back(std::vector<std::string>());

      get_raw_contents(all_directory_path, file_name, file_contents.back());

      if (file_name == last) break;
      ++day;
    }
  } else if (file_sort.substr(0, 3) == "IGU" || file_sort.find("Ultra") != std::string::npos) {  // In case of UR
    ur_flag = UR_UNKNOWN;
    std::string file_header, file_footer;
    int gps_week = 0, day = 0;
    int hour = 0;
    for (int i = 0; i < (int)first.size(); ++i) {
      int n = first.at(i);
      // Looking for the number of file name
      if ((int)'0' <= n && n < (int)('0' + 10)) {
        file_header = first.substr(0, i);
        gps_week = stoi(first.substr(file_header.size(), 4));
        day = stoi(first.substr(file_header.size() + 4, 1));
        hour = stoi(first.substr(file_header.size() + 6, 2));
        file_footer = first.substr(file_header.size() + 8);
        break;
      }
    }

    file_contents.clear();

    while (true) {
      if (hour == 24) {
        hour = 0;
        ++day;
      }
      if (day == 7) {
        ++gps_week;
        day = 0;
      }
      std::string file_name = file_header + std::to_string(gps_week) + std::to_string(day) + "_";
      if (hour < 10) {
        file_name += "0";
      }
      file_name += std::to_string(hour) + file_footer;
      file_contents.push_back(std::vector<std::string>());

      get_raw_contents(all_directory_path, file_name, file_contents.back());

      if (file_name == last) break;
      hour += 6;
    }
  } else {
    std::string file_header, file_footer;
    int gps_week = 0, day = 0;
    for (int i = 0; i < (int)first.size(); ++i) {
      int n = first.at(i);
      // Looking for the number of file name
      if ((int)'0' <= n && n < (int)('0' + 10)) {
        file_header = first.substr(0, i);
        gps_week = stoi(first.substr(file_header.size(), 4));
        day = stoi(first.substr(file_header.size() + 4, 1));
        file_footer = first.substr(file_header.size() + 5);
        break;
      }
    }

    file_contents.clear();

    while (true) {
      if (day == 7) {
        ++gps_week;
        day = 0;
      }
      std::string file_name = file_header + std::to_string(gps_week) + std::to_string(day) + file_footer;
      file_contents.push_back(std::vector<std::string>());

      get_raw_contents(all_directory_path, file_name, file_contents.back());

      if (file_name == last) break;
      ++day;
    }
  }

  return;
}

void get_clk_file_contents(std::string directory_path, std::string extension, std::string file_sort, std::string first, std::string last,
                           std::vector<std::vector<std::string>>& file_contents) {
  std::string all_directory_path = directory_path + return_dirctory_path(file_sort) + extension.substr(1) + '/';

  if (file_sort.find("Ultra") != std::string::npos) {
    std::string file_header, file_footer;
    int gps_week = 0, day = 0;
    int hour = 0;
    for (int i = 0; i < (int)first.size(); ++i) {
      int n = first.at(i);
      // Looking for the number of file name
      if ((int)'0' <= n && n < (int)('0' + 10)) {
        file_header = first.substr(0, i);
        gps_week = stoi(first.substr(file_header.size(), 4));
        day = stoi(first.substr(file_header.size() + 4, 1));
        hour = stoi(first.substr(file_header.size() + 6, 2));
        file_footer = first.substr(file_header.size() + 8);
        break;
      }
    }

    file_contents.clear();

    while (true) {
      if (hour == 24) {
        hour = 0;
        ++day;
      }
      if (day == 7) {
        ++gps_week;
        day = 0;
      }
      std::string file_name = file_header + std::to_string(gps_week) + std::to_string(day) + "_";
      if (hour < 10) {
        file_name += "0";
      }
      file_name += std::to_string(hour) + file_footer;
      file_contents.push_back(std::vector<std::string>());

      get_raw_contents(all_directory_path, file_name, file_contents.back());

      if (file_name == last) break;
      hour += 6;
    }
  } else {
    std::string file_header, file_footer;
    int gps_week = 0, day = 0;
    for (int i = 0; i < (int)first.size(); ++i) {
      int n = first.at(i);
      // Looking for the number of file name
      if ((int)'0' <= n && n < (int)('0' + 10)) {
        file_header = first.substr(0, i);
        gps_week = stoi(first.substr(file_header.size(), 4));
        day = stoi(first.substr(file_header.size() + 4, 1));
        file_footer = first.substr(file_header.size() + 5);
        break;
      }
    }

    file_contents.clear();

    while (true) {
      if (day == 7) {
        ++gps_week;
        day = 0;
      }
      std::string file_name = file_header + std::to_string(gps_week) + std::to_string(day) + file_footer;
      file_contents.push_back(std::vector<std::string>());

      get_raw_contents(all_directory_path, file_name, file_contents.back());

      if (file_name == last) break;
      ++day;
    }
  }

  return;
}

GnssSatellites* InitGnssSatellites(std::string file_name) {
  IniAccess ini_file(file_name);
  char section[] = "GNSS_SATELLIES";
  GnssSatellites* gnss_satellites = new GnssSatellites(ini_file.ReadEnable(section, "calculation"));
  if (!gnss_satellites->IsCalcEnabled()) {
    return gnss_satellites;
  }

  std::string directory_path = ini_file.ReadString(section, "directory_path");

  std::vector<std::vector<std::string>> true_position_file;
  UR_KINDS true_position_ur_flag = UR_NOT_UR;
  get_sp3_file_contents(directory_path, ini_file.ReadString(section, "true_position_file_sort"), ini_file.ReadString(section, "true_position_first"),
                        ini_file.ReadString(section, "true_position_last"), true_position_file, true_position_ur_flag);
  int true_position_interpolation_method = ini_file.ReadInt(section, "true_position_interpolation_method");
  int true_position_interpolation_number = ini_file.ReadInt(section, "true_position_interpolation_number");

  std::vector<std::vector<std::string>> true_clock_file;
  UR_KINDS true_clock_ur_flag = UR_NOT_UR;
  std::string true_clock_file_extension = ini_file.ReadString(section, "true_clock_file_extension");
  if (true_clock_file_extension == ".sp3") {
    get_sp3_file_contents(directory_path, ini_file.ReadString(section, "true_clock_file_sort"), ini_file.ReadString(section, "true_clock_first"),
                          ini_file.ReadString(section, "true_clock_last"), true_clock_file, true_clock_ur_flag);
  } else {
    get_clk_file_contents(directory_path, true_clock_file_extension, ini_file.ReadString(section, "true_clock_file_sort"),
                          ini_file.ReadString(section, "true_clock_first"), ini_file.ReadString(section, "true_clock_last"), true_clock_file);
  }
  int true_clock_interpolation_number = ini_file.ReadInt(section, "true_clock_interpolation_number");

  std::vector<std::vector<std::string>> estimate_position_file;
  UR_KINDS estimate_position_ur_flag = UR_NOT_UR;
  get_sp3_file_contents(directory_path, ini_file.ReadString(section, "estimate_position_file_sort"),
                        ini_file.ReadString(section, "estimate_position_first"), ini_file.ReadString(section, "estimate_position_last"),
                        estimate_position_file, estimate_position_ur_flag);
  int estimate_position_interpolation_method = ini_file.ReadInt(section, "estimate_position_interpolation_method");
  int estimate_position_interpolation_number = ini_file.ReadInt(section, "estimate_position_interpolation_number");
  if (estimate_position_ur_flag != UR_NOT_UR) {
    std::string ur_flag = ini_file.ReadString(section, "estimate_ur_observe_or_predict");
    if (ur_flag.find("observe") != std::string::npos) {
      estimate_position_ur_flag = (UR_KINDS)((int)UR_OBSERVE1 + (ur_flag.back() - '1'));
    } else {
      estimate_position_ur_flag = (UR_KINDS)((int)UR_PREDICT1 + (ur_flag.back() - '1'));
    }
  }

  std::vector<std::vector<std::string>> estimate_clock_file;
  UR_KINDS estimate_clock_ur_flag = estimate_position_ur_flag;
  std::string estimate_clock_file_extension = ini_file.ReadString(section, "estimate_clock_file_extension");
  if (estimate_clock_file_extension == ".sp3") {
    get_sp3_file_contents(directory_path, ini_file.ReadString(section, "estimate_clock_file_sort"),
                          ini_file.ReadString(section, "estimate_clock_first"), ini_file.ReadString(section, "estimate_clock_last"),
                          estimate_clock_file, estimate_clock_ur_flag);
  } else {
    get_clk_file_contents(directory_path, estimate_clock_file_extension, ini_file.ReadString(section, "estimate_clock_file_sort"),
                          ini_file.ReadString(section, "estimate_clock_first"), ini_file.ReadString(section, "estimate_clock_last"),
                          estimate_clock_file);
  }
  int estimate_clock_interpolation_number = ini_file.ReadInt(section, "estimate_clock_interpolation_number");

  gnss_satellites->Init(true_position_file, true_position_interpolation_method, true_position_interpolation_number, true_position_ur_flag,

                        true_clock_file, true_clock_file_extension, true_clock_interpolation_number, true_clock_ur_flag,

                        estimate_position_file, estimate_position_interpolation_method, estimate_position_interpolation_number,
                        estimate_position_ur_flag,

                        estimate_clock_file, estimate_clock_file_extension, estimate_clock_interpolation_number, estimate_clock_ur_flag);

  return gnss_satellites;
}
