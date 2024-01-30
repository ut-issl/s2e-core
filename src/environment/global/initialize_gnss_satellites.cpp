/**
 *@file initialize_gnss_satellites.cpp
 *@brief Initialize functions for GnssSatellites class
 */

#include "initialize_gnss_satellites.hpp"

#include <iostream>
#include <library/initialize/initialize_file_access.hpp>
#include <string>

/**
 *@fn ReturnDirectoryPathWithFileType
 *@brief Return directory path with file type infomation
 *@param [in] sort: File type
 *@return Directory path with file type infomation
 */
std::string ReturnDirectoryPathWithFileType(std::string sort) {
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

/**
 *@fn ReadFileContents
 *@brief Read file and convert to a vector of one line strings
 *@param [in] directory_path: Directory path of the file
 *@param [in] file_name: File name
 *@param [out] storage: vector of one line strings
 */
void ReadFileContents(std::string directory_path, std::string file_name, std::vector<std::string>& storage) {
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

/**
 *@fn ReadSp3Files
 *@brief Read multiple SP3 files in the directory and generate multiple vectors of one line strings
 *@param [in] directory_path: Directory path of the file
 *@param [in] file_sort: File type
 *@param [in] first: The first SP3 file name
 *@param [in] last: The last SP3 file name
 *@param [out] file_contents: Generated files as multiple vectors of one line strings
 */
void ReadSp3Files(std::string directory_path, std::string file_sort, std::string first, std::string last,
                  std::vector<std::vector<std::string>>& file_contents) {
  std::string all_directory_path = directory_path + ReturnDirectoryPathWithFileType(file_sort);

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

      ReadFileContents(all_directory_path, file_name, file_contents.back());

      if (file_name == last) break;
      ++day;
    }
  } else if (file_sort.substr(0, 3) == "IGU" || file_sort.find("Ultra") != std::string::npos) {  // In case of UR
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

      ReadFileContents(all_directory_path, file_name, file_contents.back());

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

      ReadFileContents(all_directory_path, file_name, file_contents.back());

      if (file_name == last) break;
      ++day;
    }
  }

  return;
}

/**
 *@fn ReadClockFiles
 *@brief Read multiple SP3 files in the directory and generate multiple vectors of one line strings
 *@param [in] directory_path: Directory path of the file
 *@param [in] extension: Extensions of the file
 *@param [in] file_sort: File type
 *@param [in] first: The first SP3 file name
 *@param [in] last: The last SP3 file name
 *@param [out] file_contents: Generated files as multiple vectors of one line strings
 */
void ReadClockFiles(std::string directory_path, std::string extension, std::string file_sort, std::string first, std::string last,
                    std::vector<std::vector<std::string>>& file_contents) {
  std::string all_directory_path = directory_path + ReturnDirectoryPathWithFileType(file_sort) + extension.substr(1) + '/';

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

      ReadFileContents(all_directory_path, file_name, file_contents.back());

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

      ReadFileContents(all_directory_path, file_name, file_contents.back());

      if (file_name == last) break;
      ++day;
    }
  }

  return;
}

GnssSatellites* InitGnssSatellites(std::string file_name) {
  IniAccess ini_file(file_name);
  char section[] = "GNSS_SATELLITES";

  const bool is_calc_enable = ini_file.ReadEnable(section, INI_CALC_LABEL);
  const bool is_log_enable = ini_file.ReadEnable(section, INI_LOG_LABEL);

  GnssSatellites* gnss_satellites = new GnssSatellites(is_calc_enable, is_log_enable);
  if (!gnss_satellites->IsCalcEnabled()) {
    return gnss_satellites;
  }

  std::string directory_path = ini_file.ReadString(section, "directory_path");
  std::string file_name_header = ini_file.ReadString(section, "file_name_header");

  std::string start_date = ini_file.ReadString(section, "start_date");
  std::string end_date = ini_file.ReadString(section, "end_date");

  // Position
  std::vector<std::vector<std::string>> position_file;
  ReadSp3Files(directory_path, file_name_header, start_date, end_date, position_file);

  // Clock
  std::string clock_file_extension = ini_file.ReadString(section, "clock_file_extension");
  std::vector<std::vector<std::string>> clock_file;
  if (clock_file_extension == "SP3") {
    ReadSp3Files(directory_path, file_name_header, start_date, end_date, clock_file);
  } else {
    ReadClockFiles(directory_path, clock_file_extension, file_name_header, start_date, end_date, clock_file);
  }

  // Initialize GNSS satellites
  gnss_satellites->Initialize(position_file, clock_file, clock_file_extension);

  return gnss_satellites;
}
