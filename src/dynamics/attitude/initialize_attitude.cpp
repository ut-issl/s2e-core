/**
 * @file initialize_attitude.hpp
 * @brief Initialize function for attitude
 */
#include "initialize_attitude.hpp"

#include <setting_file_reader/initialize_file_access.hpp>

Attitude* InitAttitude(std::string file_name, const Orbit* orbit, const LocalCelestialInformation* local_celestial_information,
                       const double step_width_s, const libra::Matrix<3, 3>& inertia_tensor_kgm2, const int spacecraft_id) {
  IniAccess ini_file(file_name);
  const char* section_ = "ATTITUDE";
  std::string mc_name = "attitude" + std::to_string(spacecraft_id);
  Attitude* attitude;

  const std::string propagate_mode = ini_file.ReadString(section_, "propagate_mode");
  const std::string initialize_mode = ini_file.ReadString(section_, "initialize_mode");

  if (propagate_mode == "RK4" && initialize_mode == "MANUAL") {
    // RK4 propagator
    libra::Vector<3> omega_b;
    ini_file.ReadVector(section_, "initial_angular_velocity_b_rad_s", omega_b);
    libra::Quaternion quaternion_i2b;
    ini_file.ReadQuaternion(section_, "initial_quaternion_i2b", quaternion_i2b);
    libra::Vector<3> torque_b;
    ini_file.ReadVector(section_, "initial_torque_b_Nm", torque_b);

    attitude = new AttitudeRk4(omega_b, quaternion_i2b, inertia_tensor_kgm2, torque_b, step_width_s, mc_name);
  } else if (propagate_mode == "RK4" && initialize_mode == "CONTROLLED") {
    // Initialize with Controlled attitude (attitude_tmp temporary used)
    IniAccess ini_file_ca(file_name);
    const char* section_ca_ = "CONTROLLED_ATTITUDE";
    const std::string main_mode_in = ini_file.ReadString(section_ca_, "main_mode");
    const std::string sub_mode_in = ini_file.ReadString(section_ca_, "sub_mode");

    AttitudeControlMode main_mode = ConvertStringToCtrlMode(main_mode_in);
    AttitudeControlMode sub_mode = ConvertStringToCtrlMode(sub_mode_in);
    libra::Quaternion quaternion_i2b;
    ini_file_ca.ReadQuaternion(section_, "initial_quaternion_i2b", quaternion_i2b);
    libra::Vector<3> main_target_direction_b, sub_target_direction_b;
    ini_file_ca.ReadVector(section_ca_, "main_pointing_direction_b", main_target_direction_b);
    ini_file_ca.ReadVector(section_ca_, "sub_pointing_direction_b", sub_target_direction_b);
    std::string mc_name_temp = section_ + std::to_string(spacecraft_id) + "_TEMP";
    Attitude* attitude_temp = new ControlledAttitude(main_mode, sub_mode, quaternion_i2b, main_target_direction_b, sub_target_direction_b,
                                                     inertia_tensor_kgm2, local_celestial_information, orbit, mc_name_temp);
    attitude_temp->Propagate(step_width_s);
    quaternion_i2b = attitude_temp->GetQuaternion_i2b();
    libra::Vector<3> omega_b = libra::Vector<3>(0.0);
    libra::Vector<3> torque_b = libra::Vector<3>(0.0);

    attitude = new AttitudeRk4(omega_b, quaternion_i2b, inertia_tensor_kgm2, torque_b, step_width_s, mc_name);
  } else if (propagate_mode == "CONTROLLED") {
    // Controlled attitude
    IniAccess ini_file_ca(file_name);
    const char* section_ca_ = "CONTROLLED_ATTITUDE";
    const std::string main_mode_in = ini_file.ReadString(section_ca_, "main_mode");
    const std::string sub_mode_in = ini_file.ReadString(section_ca_, "sub_mode");

    AttitudeControlMode main_mode = ConvertStringToCtrlMode(main_mode_in);
    AttitudeControlMode sub_mode = ConvertStringToCtrlMode(sub_mode_in);
    libra::Quaternion quaternion_i2b;
    ini_file_ca.ReadQuaternion(section_, "initial_quaternion_i2b", quaternion_i2b);
    libra::Vector<3> main_target_direction_b, sub_target_direction_b;
    ini_file_ca.ReadVector(section_ca_, "main_pointing_direction_b", main_target_direction_b);
    ini_file_ca.ReadVector(section_ca_, "sub_pointing_direction_b", sub_target_direction_b);

    attitude = new ControlledAttitude(main_mode, sub_mode, quaternion_i2b, main_target_direction_b, sub_target_direction_b, inertia_tensor_kgm2,
                                      local_celestial_information, orbit, mc_name);
  } else {
    std::cerr << "ERROR: attitude propagation mode: " << propagate_mode << " is not defined!" << std::endl;
    std::cerr << "The attitude mode is automatically set as RK4" << std::endl;

    libra::Vector<3> omega_b;
    ini_file.ReadVector(section_, "initial_angular_velocity_b_rad_s", omega_b);
    libra::Quaternion quaternion_i2b;
    ini_file.ReadQuaternion(section_, "initial_quaternion_i2b", quaternion_i2b);
    libra::Vector<3> torque_b;
    ini_file.ReadVector(section_, "initial_torque_b_Nm", torque_b);

    attitude = new AttitudeRk4(omega_b, quaternion_i2b, inertia_tensor_kgm2, torque_b, step_width_s, mc_name);
  }

  return attitude;
}
