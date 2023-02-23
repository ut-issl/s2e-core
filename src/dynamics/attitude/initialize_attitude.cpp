/**
 * @file initialize_attitude.hpp
 * @brief Initialize function for attitude
 */
#include "initialize_attitude.hpp"

#include <library/initialize/initialize_file_access.hpp>

Attitude* InitAttitude(std::string file_name, const Orbit* orbit, const LocalCelestialInformation* celes_info, const double step_sec,
                       const Matrix<3, 3> inertia_tensor, const int sat_id) {
  IniAccess ini_file(file_name);
  const char* section_ = "ATTITUDE";
  std::string mc_name = section_ + std::to_string(sat_id);  // FIXME
  Attitude* attitude;

  const std::string propagate_mode = ini_file.ReadString(section_, "propagate_mode");

  if (propagate_mode == "RK4") {
    // RK4 propagator
    Vector<3> omega_b;
    ini_file.ReadVector(section_, "initial_angular_velocity_b_rad_s", omega_b);
    Quaternion quaternion_i2b;
    ini_file.ReadQuaternion(section_, "initial_quaternion_i2b", quaternion_i2b);
    Vector<3> torque_b;
    ini_file.ReadVector(section_, "initial_torque_b_Nm", torque_b);

    attitude = new AttitudeRk4(omega_b, quaternion_i2b, inertia_tensor, torque_b, step_sec, mc_name);
  } else if (propagate_mode == "CONTROLLED") {
    // Controlled attitude
    IniAccess ini_file_ca(file_name);
    const char* section_ca_ = "CONTROLLED_ATTITUDE";
    const std::string main_mode_in = ini_file.ReadString(section_ca_, "main_mode");
    const std::string sub_mode_in = ini_file.ReadString(section_ca_, "sub_mode");

    AttCtrlMode main_mode = ConvertStringToCtrlMode(main_mode_in);
    AttCtrlMode sub_mode = ConvertStringToCtrlMode(sub_mode_in);
    Quaternion quaternion_i2b;
    ini_file_ca.ReadQuaternion(section_, "initial_quaternion_i2b", quaternion_i2b);
    Vector<3> main_target_direction_b, sub_target_direction_b;
    ini_file_ca.ReadVector(section_ca_, "main_pointing_direction_b", main_target_direction_b);
    ini_file_ca.ReadVector(section_ca_, "sub_pointing_direction_b", sub_target_direction_b);
    attitude = new ControlledAttitude(main_mode, sub_mode, quaternion_i2b, main_target_direction_b, sub_target_direction_b, inertia_tensor,
                                      celes_info, orbit, mc_name);
  } else {
    std::cerr << "ERROR: attitude propagation mode: " << propagate_mode << " is not defined!" << std::endl;
    std::cerr << "The attitude mode is automatically set as RK4" << std::endl;

    Vector<3> omega_b;
    ini_file.ReadVector(section_, "initial_angular_velocity_b_rad_s", omega_b);
    Quaternion quaternion_i2b;
    ini_file.ReadQuaternion(section_, "initial_quaternion_i2b", quaternion_i2b);
    Vector<3> torque_b;
    ini_file.ReadVector(section_, "initial_torque_b_Nm", torque_b);

    attitude = new AttitudeRk4(omega_b, quaternion_i2b, inertia_tensor, torque_b, step_sec, mc_name);
  }

  return attitude;
}
