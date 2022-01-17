#include <iostream>
#include <sstream>

#include "../../Interface/LogOutput/LogUtility.h"
#include "CelestialInformation.h"
#include "SpiceUsr.h"

using namespace std;

CelestialInformation::CelestialInformation(string inertial_frame,
                                           string aber_cor, string center_obj,
                                           RotationMode rotation_mode,
                                           int num_of_selected_body,
                                           int *selected_body)
    : inertial_frame_(inertial_frame), aber_cor_(aber_cor),
      center_obj_(center_obj), rotation_mode_(rotation_mode),
      num_of_selected_body_(num_of_selected_body),
      selected_body_(selected_body) {
  int num_of_state = num_of_selected_body_ * 3;
  celes_objects_pos_from_center_i_ = new double[num_of_state];
  celes_objects_vel_from_center_i_ = new double[num_of_state];
  celes_objects_gravity_constant_ = new double[num_of_selected_body_];

  // Acquisition of gravity constant
  for (int i = 0; i < num_of_selected_body_; i++) {
    SpiceInt planet_id = selected_body_[i];
    SpiceInt dim;
    SpiceDouble gravity_constant;
    bodvcd_c(planet_id, "GM", 1, &dim, &gravity_constant);
    // CONVERT FROM [km^3/s^2] to [m^3/s^2]
    celes_objects_gravity_constant_[i] = gravity_constant * 1E+9;
  }

  EarthRotation_ = new CelestialRotation(rotation_mode_, center_obj_);
}

CelestialInformation::CelestialInformation(const CelestialInformation &obj)
    : inertial_frame_(obj.inertial_frame_), aber_cor_(obj.aber_cor_),
      center_obj_(obj.center_obj_), rotation_mode_(obj.rotation_mode_),
      num_of_selected_body_(obj.num_of_selected_body_) {
  int num_of_state = num_of_selected_body_ * 3;
  int sd = sizeof(double);
  int si = sizeof(int);

  selected_body_ = new int[num_of_selected_body_];
  celes_objects_pos_from_center_i_ = new double[num_of_state];
  celes_objects_vel_from_center_i_ = new double[num_of_state];
  celes_objects_gravity_constant_ = new double[num_of_selected_body_];

  memcpy(selected_body_, obj.selected_body_, si * num_of_selected_body_);
  memcpy(celes_objects_pos_from_center_i_, obj.celes_objects_pos_from_center_i_,
         sd * num_of_state);
  memcpy(celes_objects_vel_from_center_i_, obj.celes_objects_vel_from_center_i_,
         sd * num_of_state);
  memcpy(celes_objects_gravity_constant_, obj.celes_objects_gravity_constant_,
         sd * num_of_selected_body_);
}

CelestialInformation::~CelestialInformation() {
  delete[] celes_objects_pos_from_center_i_;
  delete[] celes_objects_vel_from_center_i_;
  delete[] celes_objects_gravity_constant_;
  delete[] selected_body_;
  delete EarthRotation_;
}

void CelestialInformation::UpdateAllObjectsInfo(const double current_jd) {
  SpiceDouble rv_buf[6], lt, et;
  SpiceBoolean found;
  const int maxlen = 100;
  char namebuf[maxlen];
  string jd = "jd " + to_string(current_jd);
  str2et_c(jd.c_str(), &et);

  for (int i = 0; i < num_of_selected_body_; i++) {
    SpiceInt planet_id = selected_body_[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean *)&found);
    // Acquisition of position and velocity
    spkezr_c(namebuf, et, inertial_frame_.c_str(), aber_cor_.c_str(),
             center_obj_.c_str(), (SpiceDouble *)rv_buf, (SpiceDouble *)&lt);
    // CONVERT [km], [km/s] to [m], [m/s]
    for (int j = 0; j < 3; j++) {
      celes_objects_pos_from_center_i_[i * 3 + j] = rv_buf[j] * 1000.0;
      celes_objects_vel_from_center_i_[i * 3 + j] = rv_buf[j + 3] * 1000.0;
    }
  }

  // Update CelesRot
  EarthRotation_->Update(current_jd);
}

Vector<3> CelestialInformation::GetPosFromCenter_i(const int id) const {
  Vector<3> pos(0.0);
  if (id > num_of_selected_body_)
    return pos;
  for (int i = 0; i < 3; i++)
    pos[i] = celes_objects_pos_from_center_i_[id * 3 + i];
  return pos;
}

Vector<3> CelestialInformation::GetVelFromCenter_i(const int id) const {
  Vector<3> vel(0.0);
  if (id > num_of_selected_body_)
    return vel;
  for (int i = 0; i < 3; i++)
    vel[i] = celes_objects_vel_from_center_i_[id * 3 + i];
  return vel;
}

Vector<3>
CelestialInformation::GetPosFromCenter_i(const char *body_name) const {
  int id = CalcBodyIdFromName(body_name);
  return GetPosFromCenter_i(id);
}

Vector<3>
CelestialInformation::GetVelFromCenter_i(const char *body_name) const {
  int id = CalcBodyIdFromName(body_name);
  return GetVelFromCenter_i(id);
}

double CelestialInformation::GetGravityConstant(const char *body_name) const {
  int index = CalcBodyIdFromName(body_name);
  return celes_objects_gravity_constant_[index];
}

int CelestialInformation::CalcBodyIdFromName(const char *body_name) const {
  int index = 0;
  SpiceInt planet_id;
  SpiceBoolean found;
  const int maxlen = 100;
  // Acquisition of ID from body name
  bodn2c_c(body_name, (SpiceInt *)&planet_id, (SpiceBoolean *)&found);
  for (int i = 0; i < num_of_selected_body_; i++) {
    if (selected_body_[i] == planet_id) {
      index = i;
      break;
    }
  }
  return index;
}

string CelestialInformation::GetLogHeader() const {
  SpiceBoolean found;
  const int maxlen = 100;
  char namebuf[maxlen];
  string str_tmp = "";
  for (int i = 0; i < num_of_selected_body_; i++) {
    SpiceInt planet_id = selected_body_[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean *)&found);
    string name = namebuf;
    string body_pos = name + "_pos";
    string body_vel = name + "_vel";
    //　OUTPUT ONLY POS/VEL LOOKED FROM S/C AT THIS MOMENT
    str_tmp += WriteVector(body_pos, "i", "m", 3);
    str_tmp += WriteVector(body_vel, "i", "m/s", 3);
  }
  return str_tmp;
}

string CelestialInformation::GetLogValue() const {
  string str_tmp = "";
  for (int i = 0; i < num_of_selected_body_; i++) {
    //　OUTPUT ONLY POS/VEL LOOKED FROM S/C AT THIS MOMENT
    for (int j = 0; j < 3; j++) {
      str_tmp += WriteScalar(celes_objects_pos_from_center_i_[i * 3 + j]);
    }
    for (int j = 0; j < 3; j++) {
      str_tmp += WriteScalar(celes_objects_vel_from_center_i_[i * 3 + j]);
    }
  }
  return str_tmp;
}

void CelestialInformation::DebugOutput(void) {
  SpiceBoolean found;
  const int maxlen = 100;
  char namebuf[maxlen];
  cout << "BODY NAME, POSx,y,z[m], VELx,y,z[m/s] from CENTER;\nPOSx,y,z[m], "
          "VELx,y,z[m/s] from SC";
  for (int i = 0; i < num_of_selected_body_; i++) {
    SpiceInt planet_id = selected_body_[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean *)&found);
    //		cout<<namebuf<<
  }
  cout << "GRAVITY CONSTASNT of\n";
  for (int i = 0; i < num_of_selected_body_; i++) {
    SpiceInt planet_id = selected_body_[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean *)&found);
    cout << namebuf << "is"
         << ": " << celes_objects_gravity_constant_[i] << "\n";
  }
}
