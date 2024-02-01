/**
 *@file initialize_gnss_satellites.cpp
 *@brief Initialize functions for GnssSatellites class
 */

#include "initialize_gnss_satellites.hpp"

#include <iostream>
#include <library/gnss/igs_product_name_handling.hpp>
#include <library/gnss/sp3_file_reader.hpp>
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

  const std::string directory_path = ini_file.ReadString(section, "directory_path");
  const std::string file_name_header = ini_file.ReadString(section, "file_name_header");
  const std::string orbit_data_period = ini_file.ReadString(section, "orbit_data_period");
  const std::string clock_file_name_footer = ini_file.ReadString(section, "clock_file_name_footer");
  bool use_sp3_for_clock = false;
  if (clock_file_name_footer == (orbit_data_period + "_ORB.SP3")) {
    use_sp3_for_clock = true;
  }

  // Duration
  const size_t start_date = (size_t)ini_file.ReadInt(section, "start_date");
  const size_t end_date = (size_t)ini_file.ReadInt(section, "end_date");
  if (start_date > end_date) {
    std::cout << "[ERROR] GNSS satellite initialize: start_date is larger than the end date." << std::endl;
  }

  // Read all product files
  std::vector<Sp3FileReader> sp3_file_readers;

  size_t read_file_date = start_date;
  while (read_file_date <= end_date) {
    std::string sp3_file_name = GetOrbitClockFinalFileName(file_name_header, read_file_date, orbit_data_period);
    std::string sp3_full_file_path = directory_path + "/" + sp3_file_name;

    // Read SP3
    sp3_file_readers.push_back(Sp3FileReader(sp3_full_file_path));

    // Clock file
    if (!use_sp3_for_clock) {
      std::string clk_file_name =
          GetOrbitClockFinalFileName(file_name_header, read_file_date, clock_file_name_footer.substr(0, 3), clock_file_name_footer.substr(4, 7));
      std::string clk_full_file_path = directory_path + "/" + clk_file_name;
      // TODO: Read CLK
    }

    // Increment
    read_file_date = IncrementYearDoy(read_file_date);
  }

  // Old descriptions TODO: delete
  std::vector<std::vector<std::string>> position_file;
  std::vector<std::vector<std::string>> clock_file;
  gnss_satellites->Initialize(position_file, clock_file, clock_file_name_footer);

  return gnss_satellites;
}
