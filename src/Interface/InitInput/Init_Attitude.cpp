#include "Initialize.h"
#include "../../Dynamics/Attitude/AttitudeRK4.h"
#include "../../Dynamics/Attitude/ControlledAttitude.h"

Attitude* InitAttitude(string file_name, const Orbit* orbit, const CelestialInformation* celes_info)
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
    double prop_step = ini_file.ReadDouble(section_, "PropStepSec");
    Vector<9> inertia_vec;
    ini_file.ReadVector(section_, "Iner", inertia_vec);
    Matrix<3, 3> inertia_tensor;
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        inertia_tensor[i][j] = inertia_vec[i * 3 + j];
      }
    }
    string name = section_; // "Attitude" + to_string(id);
    attitude = new AttitudeRK4(omega_b, quaternion_i2b, inertia_tensor, torque_b, prop_step, name);
  }
  // Controlled attitude
  else
  {
    //new file open
    string f_name_ca = ini_file.ReadString(section_,"ControlledAttitude_file");
    IniAccess ini_file_ca(f_name_ca);
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