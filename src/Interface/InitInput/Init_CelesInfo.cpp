#include <Environment/Global/CelestialInformation.h>
#include <SpiceUsr.h>
#include <stdlib.h>

#include <cassert>

#include "Initialize.h"

CelestialInformation* InitCelesInfo(std::string file_name) {
  IniAccess ini_file(file_name);
  char* section = "PLANET_SELECTION";
  char* furnsh_section = "FURNSH_PATH";

  //各種定義読み込み
  std::string inertial_frame = ini_file.ReadString(section, "inertial_frame");
  std::string aber_cor = ini_file.ReadString(section, "aberration_correction");
  std::string center_obj = ini_file.ReadString(section, "center_object");
  RotationMode rotation_mode;
  std::string rotation_mode_temp =
      ini_file.ReadString(section, "rotation_mode");
  if (rotation_mode_temp == "Idle") {
    rotation_mode = Idle;
  } else if (rotation_mode_temp == "Simple") {
    rotation_mode = Simple;
  } else if (rotation_mode_temp == "Full") {
    rotation_mode = Full;
  } else  // if rotation_mode is neither Idle, Simple, nor Full, set
          // rotation_mode to Idle
  {
    rotation_mode = Idle;
  }

  // SPICE Furnsh
  std::vector<std::string> keywords = {"TLS", "TPC1", "TPC2", "TPC3", "BSP"};
  for (int i = 0; i < keywords.size(); i++) {
    std::string fname =
        ini_file.ReadString(furnsh_section, keywords[i].c_str());
    furnsh_c(fname.c_str());
  }

  //天体情報をinitialize
  const int num_of_selected_body =
      ini_file.ReadInt(section, "num_of_selected_body");
  int* selected_body = new int
      [num_of_selected_body];  // これのdeleteはCelestialInformationに任せる
  SpiceInt planet_id;
  SpiceBoolean found;
  for (int i = 0; i < num_of_selected_body; i++) {
    std::string selected_body_i = "selected_body(" + std::to_string(i) + ")";
    char selected_body_temp[30];
    ini_file.ReadChar(section, selected_body_i.c_str(), 30, selected_body_temp);
    bodn2c_c(selected_body_temp, (SpiceInt*)&planet_id, (SpiceBoolean*)&found);
    std::string body_name = selected_body_temp;

    // If the object specified in the ini file is not found, exit the program.
    assert(found == SPICETRUE);

    selected_body[i] = planet_id;
  }
  CelestialInformation* celestial_info;
  celestial_info = new CelestialInformation(
      inertial_frame, aber_cor, center_obj, rotation_mode, num_of_selected_body,
      selected_body);

  // log setting
  celestial_info->IsLogEnabled = ini_file.ReadEnable(section, LOG_LABEL);

  return celestial_info;
}
