#include <iostream>
#include "../../Environment/Global/GnssSatellites.h"
#include "Initialize.h"

string return_dirctory_path(string sort)
{
  string main_directory,sub_directory;

  if(sort.substr(0, 2) == "IG"){
    main_directory = "IGS/";
    if(sort.at(2) == 'S'){
      sub_directory = "igs/";
    }else if(sort.at(2) == 'R'){
      sub_directory = "igr/";
    }else if(sort.at(2) == 'U'){
      sub_directory = "igu/";
    }
  }else if(sort.substr(0, 2) == "ma"){
    main_directory = "JAXA/";
    sub_directory = "madoca/";
  }else{
    for(int i = 0;i < sort.size();++i){
      if(sort.at(i) == '_'){
        main_directory = sort.substr(0, i) + "/";
        if(sort.at(i + 1) == 'F'){
          sub_directory = "final/";
        }else if(sort.at(i + 1) == 'R'){
          sub_directory = "rapid/";
        }else if(sort.at(i + 1) == 'U'){
          sub_directory = "ultra_rapid/";
        }else{
          cout << "file_sort has something wrong" << endl;
          exit(1);
        }
        break;
      }
    }
  }

  return main_directory + sub_directory;
}

void get_raw_contents(string directory_path, string file_name, vector<string>& storage)
{
  string all_file_path = directory_path + file_name;
  ifstream ifs(all_file_path);

  if(!ifs.is_open()){
    cout << "in " << directory_path << "gnss file: " << file_name << " not found" << endl;
    exit(1);
  }
  string str;
  while(getline(ifs, str)){
    storage.push_back(str);
  }
  ifs.close();
  if(storage.back() == "EOF") storage.pop_back();

  return;
}

void get_sp3_file_contents(string directory_path, string file_sort, string first, string last, vector<vector<string>>& file_contents, bool& ur_flag)
{
  string all_directory_path = directory_path + return_dirctory_path(file_sort);
  ur_flag = false;

  if(first.substr(0, 3) == "COD"){
    string file_header = "COD0MGXFIN_"; 
    string file_footer = "0000_01D_05M_ORB.SP3";
    int year = stoi(first.substr(file_header.size(), 4));
    int year_last_day = 365 + (year%4 == 0) - (year%100 == 0) + (year%400 == 0);
    int day = stoi(first.substr(file_header.size() + 4, 3));

    file_contents.clear();

    while(true){
      if(day > year_last_day){
        ++year;
        year_last_day = 365 + (year%4 == 0) - (year%100 == 0) + (year%400 == 0);
        day = 1;
      }
      string s_day;
      if(day >= 100) s_day = to_string(day);
      else if(day >= 10) s_day = '0' + to_string(day);
      else s_day = "00" + to_string(day);
      string file_name = file_header + to_string(year) + s_day + file_footer;
      file_contents.push_back(vector<string>());

      get_raw_contents(all_directory_path, file_name, file_contents.back());
      
      if(file_name == last) break;
      ++day;
    }
  }else if(file_sort.substr(0, 3) == "IGU" || (file_sort.size() >= 11 && file_sort.substr(file_sort.size() - 11) == "Ultra_rapid")){ //URの場合
    ur_flag = true;
    string file_header, file_footer;
    int gps_week, day;
    int hour;
    for(int i = 0;i < first.size();++i){
      int n = first.at(i);
      if(48 <= n && n < 58){
        file_header = first.substr(0, i);
        gps_week = stoi(first.substr(file_header.size(), 4));
        day = stoi(first.substr(file_header.size() + 4, 1));
        hour = stoi(first.substr(file_header.size() + 6, 2));
        file_footer = first.substr(file_header.size() + 8);
        break;
      }
    }

    file_contents.clear();

    while(true){
      if(hour == 24){
        hour = 0;
        ++day;
      }
      if(day == 7){
        ++gps_week;
        day = 0;
      }
      string file_name = file_header + to_string(gps_week) + to_string(day) + "_";
      if(hour < 10){
        file_name += "0";
      }
      file_name += to_string(hour) + file_footer;
      file_contents.push_back(vector<string>());

      get_raw_contents(all_directory_path, file_name, file_contents.back());
      
      if(file_name == last) break;
      hour += 6;
    }
  }else{
    string file_header, file_footer;
    int gps_week, day;
    for(int i = 0;i < first.size();++i){
      int n = first.at(i);
      if(48 <= n && n < 58){
        file_header = first.substr(0, i);
        gps_week = stoi(first.substr(file_header.size(), 4));
        day = stoi(first.substr(file_header.size() + 4, 1));
        file_footer = first.substr(file_header.size() + 5);
        break;
      }
    }
    
    file_contents.clear();

    while(true){
      if(day == 7){
        ++gps_week;
        day = 0;
      }
      string file_name = file_header + to_string(gps_week) + to_string(day) + file_footer;
      file_contents.push_back(vector<string>());

      get_raw_contents(all_directory_path, file_name, file_contents.back());
      
      if(file_name == last) break;
      ++day;
    }
  }

  return;
}

void get_clk_file_contents(string directory_path, string extension, string file_sort, string first, string last, vector<vector<string>>& file_contents)
{
  string all_directory_path = directory_path + return_dirctory_path(file_sort) + extension.substr(1) + '/';
  
  string file_header, file_footer;
  int gps_week, day;
  for(int i = 0;i < first.size();++i){
    int n = first.at(i);
    if(48 <= n && n < 58){
      file_header = first.substr(0, i);
      gps_week = stoi(first.substr(file_header.size(), 4));
      day = stoi(first.substr(file_header.size() + 4, 1));
      file_footer = first.substr(file_header.size() + 5);
      break;
    }
  }

  file_contents.clear();

  while(true){
    if(day == 7){
      ++gps_week;
      day = 0;
    }
    string file_name = file_header + to_string(gps_week) + to_string(day) + file_footer;
    file_contents.push_back(vector<string>());

    get_raw_contents(all_directory_path, file_name, file_contents.back());
    
    if(file_name == last) break;
    ++day;
  }

  return;
}

GnssSatellites* InitGnssSatellites(string file_name)
{   
  IniAccess ini_file(file_name);
  char* section = "GNSS_SATELLIES";
  GnssSatellites* gnss_satellites = new GnssSatellites(ini_file.ReadEnable(section, CALC_LABEL));
  if(!gnss_satellites->IsCalcEnabled()){
    return gnss_satellites;
  }

  string directory_path = ini_file.ReadString(section, "directory_path");

  vector<vector<string>> true_position_file;
  bool true_position_ur_flag = false;
  get_sp3_file_contents(directory_path, 
                        ini_file.ReadString(section, "true_position_file_sort"), 
                        ini_file.ReadString(section, "true_position_first"),
                        ini_file.ReadString(section, "true_position_last"), true_position_file, true_position_ur_flag);
  int true_position_interpolation_method = ini_file.ReadInt(section, "true_position_interpolation_method");
  int true_position_interpolation_number = ini_file.ReadInt(section, "true_position_interpolation_number");

  vector<vector<string>> true_clock_file;
  bool true_clock_ur_flag = false;
  string true_clock_file_extension = ini_file.ReadString(section, "true_clock_file_extension");
  if(true_clock_file_extension == ".sp3"){
    get_sp3_file_contents(directory_path, 
                          ini_file.ReadString(section, "true_clock_file_sort"), 
                          ini_file.ReadString(section, "true_clock_first"),
                          ini_file.ReadString(section, "true_clock_last"), true_clock_file, true_clock_ur_flag);
  }else{
    get_clk_file_contents(directory_path,
                          true_clock_file_extension,
                          ini_file.ReadString(section, "true_clock_file_sort"),
                          ini_file.ReadString(section, "true_clock_first"),
                          ini_file.ReadString(section, "true_clock_last"),
                          true_clock_file);
  }
  int true_clock_interpolation_number = ini_file.ReadInt(section, "true_clock_interpolation_number");

  vector<vector<string>> estimate_position_file;
  bool estimate_position_ur_flag = true;
  get_sp3_file_contents(directory_path, 
                        ini_file.ReadString(section, "estimate_position_file_sort"), 
                        ini_file.ReadString(section, "estimate_position_first"),
                        ini_file.ReadString(section, "estimate_position_last"), estimate_position_file, estimate_position_ur_flag);
  int estimate_position_interpolation_method = ini_file.ReadInt(section, "estimate_position_interpolation_method");
  int estimate_position_interpolation_number = ini_file.ReadInt(section, "estimate_position_interpolation_number");

  vector<vector<string>> estimate_clock_file;
  bool estimate_clock_ur_flag = false;
  string estimate_clock_file_extension = ini_file.ReadString(section, "estimate_clock_file_extension");
  if(estimate_clock_file_extension == ".sp3"){
    get_sp3_file_contents(directory_path, 
                          ini_file.ReadString(section, "estimate_clock_file_sort"), 
                          ini_file.ReadString(section, "estimate_clock_first"),
                          ini_file.ReadString(section, "estimate_clock_last"), estimate_clock_file, estimate_clock_ur_flag);
  }else{
    get_clk_file_contents(directory_path,
                          estimate_clock_file_extension,
                          ini_file.ReadString(section, "estimate_clock_file_sort"),
                          ini_file.ReadString(section, "estimate_clock_first"),
                          ini_file.ReadString(section, "estimate_clock_last"),
                          estimate_clock_file);
  }
  int estimate_clock_interpolation_number = ini_file.ReadInt(section, "estimate_clock_interpolation_number");

  gnss_satellites->Init(true_position_file,
                        true_position_interpolation_method,
                        true_position_interpolation_number,
                        true_position_ur_flag,

                        true_clock_file,
                        true_clock_file_extension,
                        true_clock_interpolation_number,
                        true_clock_ur_flag,
    
                        estimate_position_file,
                        estimate_position_interpolation_method,
                        estimate_position_interpolation_number,
                        estimate_position_ur_flag,

                        estimate_clock_file,
                        estimate_clock_file_extension,
                        estimate_clock_interpolation_number,
                        estimate_clock_ur_flag);

  return gnss_satellites;
}
