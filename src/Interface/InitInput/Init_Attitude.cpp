#include "Initialize.h"
#include <Dynamics/Attitude/AttitudeRK4.h>
#include <Dynamics/Attitude/ControlledAttitude.h>

Attitude* InitAttitude(std::string file_name, const Orbit* orbit, const LocalCelestialInformation* celes_info, const double step_sec, const Matrix<3, 3> inertia_tensor, const int sat_id)
{
  IniAccess ini_file(file_name);
  char* section_ = "ATTITUDE";
  Attitude* attitude;

  int propagate_mode = ini_file.ReadInt(section_, "propagate_mode");

  // RK4 propagator
  if (propagate_mode == 0)
  {
    Vector<3> omega_b;
    ini_file.ReadVector(section_, "Omega_b", omega_b);
    Quaternion quaternion_i2b;
    ini_file.ReadQuaternion(section_, "Quaternion_i2b", quaternion_i2b);
    Vector<3> torque_b;
    ini_file.ReadVector(section_, "Torque_b", torque_b);
  
    std::string name = section_ + std::to_string(sat_id); // "Attitude" + to_string(id);
    attitude = new AttitudeRK4(omega_b, quaternion_i2b, inertia_tensor, torque_b, step_sec, name);
  }
  // Controlled attitude
  else
  {
    //new file open
    IniAccess ini_file_ca(file_name);
    //
    char* section_ca_ = "ControlledAttitude";
    AttCtrlMode main_mode = static_cast<AttCtrlMode>(ini_file_ca.ReadInt(section_ca_, "main_mode"));
    AttCtrlMode sub_mode = static_cast<AttCtrlMode>(ini_file_ca.ReadInt(section_ca_, "sub_mode"));
    Quaternion quaternion_i2t;
    ini_file_ca.ReadQuaternion(section_ca_, "quaternion_i2t", quaternion_i2t);
    Vector<3> pointing_t_b,pointing_sub_t_b;
    ini_file_ca.ReadVector(section_ca_, "pointing_t_b", pointing_t_b);
    ini_file_ca.ReadVector(section_ca_, "pointing_sub_t_b", pointing_sub_t_b);
    attitude = new ControlledAttitude(main_mode,sub_mode,quaternion_i2t,pointing_t_b,pointing_sub_t_b,celes_info,orbit);
  }

  return attitude;
}
