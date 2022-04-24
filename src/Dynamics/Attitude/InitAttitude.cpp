#include "InitAttitude.hpp"

#include <Interface/InitInput/IniAccess.h>

Attitude* InitAttitude(std::string file_name, const Orbit* orbit, const LocalCelestialInformation* celes_info, const double step_sec,
                       const Matrix<3, 3> inertia_tensor, const int sat_id) {
  IniAccess ini_file(file_name);
  const char* section_ = "ATTITUDE";
  std::string mc_name = section_ + std::to_string(sat_id);  // "Attitude" + to_string(id);
  Attitude* attitude;

  const std::string propagate_mode = ini_file.ReadString(section_, "propagate_mode");

  if (propagate_mode == "RK4") {
    // RK4 propagator
    Vector<3> omega_b;
    ini_file.ReadVector(section_, "Omega_b", omega_b);
    Quaternion quaternion_i2b;
    ini_file.ReadQuaternion(section_, "Quaternion_i2b", quaternion_i2b);
    Vector<3> torque_b;
    ini_file.ReadVector(section_, "Torque_b", torque_b);

    attitude = new AttitudeRK4(omega_b, quaternion_i2b, inertia_tensor, torque_b, step_sec, mc_name);
  } else if (propagate_mode == "CONTROLLED") {
    // Controlled attitude
    IniAccess ini_file_ca(file_name);
    const char* section_ca_ = "ControlledAttitude";
    const std::string main_mode_in = ini_file.ReadString(section_ca_, "main_mode");
    const std::string sub_mode_in = ini_file.ReadString(section_ca_, "sub_mode");

    AttCtrlMode main_mode = ConvertStringToCtrlMode(main_mode_in);
    AttCtrlMode sub_mode = ConvertStringToCtrlMode(sub_mode_in);
    Quaternion quaternion_i2b;
    ini_file_ca.ReadQuaternion(section_, "Quaternion_i2b", quaternion_i2b);
    Vector<3> pointing_t_b, pointing_sub_t_b;
    ini_file_ca.ReadVector(section_ca_, "pointing_t_b", pointing_t_b);
    ini_file_ca.ReadVector(section_ca_, "pointing_sub_t_b", pointing_sub_t_b);
    attitude = new ControlledAttitude(main_mode, sub_mode, quaternion_i2b, pointing_t_b, pointing_sub_t_b, celes_info, orbit, mc_name);
  } else {
    std::cerr << "ERROR: attitude propagation mode: " << propagate_mode << " is not defined!" << std::endl;
  }

  return attitude;
}
