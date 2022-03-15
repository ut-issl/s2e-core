#include "LocalCelestialInformation.h"

#include <Interface/LogOutput/LogUtility.h>
#include <SpiceUsr.h>

#include <iostream>
#include <sstream>

using namespace std;

LocalCelestialInformation::LocalCelestialInformation(
    const CelestialInformation* glo_celes_info)
    : glo_celes_info_(glo_celes_info) {
  int num_of_state = glo_celes_info_->GetNumBody() * 3;
  celes_objects_pos_from_center_b_ = new double[num_of_state];
  celes_objects_vel_from_center_b_ = new double[num_of_state];
  celes_objects_pos_from_sc_i_ = new double[num_of_state];
  celes_objects_vel_from_sc_i_ = new double[num_of_state];
  celes_objects_pos_from_sc_b_ = new double[num_of_state];
  celes_objects_vel_from_sc_b_ = new double[num_of_state];

  for (int i = 0; i < num_of_state; i++) {
    celes_objects_pos_from_center_b_[i] = 0.0;
    celes_objects_vel_from_center_b_[i] = 0.0;
    celes_objects_pos_from_sc_i_[i] = 0.0;
    celes_objects_vel_from_sc_i_[i] = 0.0;
    celes_objects_pos_from_sc_b_[i] = 0.0;
    celes_objects_vel_from_sc_b_[i] = 0.0;
  }
}

LocalCelestialInformation::~LocalCelestialInformation() {
  delete[] celes_objects_pos_from_center_b_;
  delete[] celes_objects_vel_from_center_b_;
  delete[] celes_objects_pos_from_sc_i_;
  delete[] celes_objects_vel_from_sc_i_;
  delete[] celes_objects_pos_from_sc_b_;
  delete[] celes_objects_vel_from_sc_b_;
}

void LocalCelestialInformation::UpdateAllObjectsInfo(
    const Vector<3> sc_pos_from_center_i, const Vector<3> sc_vel_from_center_i,
    const Quaternion q_i2b, const Vector<3> sc_body_rate) {
  Vector<3> pos_center_i, vel_center_i;
  for (int i = 0; i < glo_celes_info_->GetNumBody(); i++) {
    pos_center_i = glo_celes_info_->GetPosFromCenter_i(i);
    vel_center_i = glo_celes_info_->GetVelFromCenter_i(i);
    // Change origin of frame
    for (int j = 0; j < 3; j++) {
      celes_objects_pos_from_sc_i_[i * 3 + j] =
          pos_center_i[j] - sc_pos_from_center_i[j];
      celes_objects_vel_from_sc_i_[i * 3 + j] =
          vel_center_i[j] - sc_vel_from_center_i[j];
    }
  }
  CalcAllPosVel_b(q_i2b, sc_body_rate);

  return;
}

void LocalCelestialInformation::CalcAllPosVel_b(const Quaternion q_i2b,
                                                const Vector<3> sc_body_rate) {
  Vector<3> pos_center_i, vel_center_i;
  double r_buf1_i[3], v_buf1_i[3], r_buf1_b[3], v_buf1_b[3];
  double r_buf2_i[3], v_buf2_i[3], r_buf2_b[3], v_buf2_b[3];
  for (int i = 0; i < glo_celes_info_->GetNumBody(); i++) {
    pos_center_i = glo_celes_info_->GetPosFromCenter_i(i);
    vel_center_i = glo_celes_info_->GetVelFromCenter_i(i);
    for (int j = 0; j < 3; j++) {
      r_buf1_i[j] = pos_center_i[j];
      r_buf2_i[j] = celes_objects_pos_from_sc_i_[i * 3 + j];
      v_buf1_i[j] = vel_center_i[j];
      v_buf2_i[j] = celes_objects_vel_from_sc_i_[i * 3 + j];
    }
    Convert_i2b(r_buf1_i, r_buf1_b, q_i2b);
    Convert_i2b(r_buf2_i, r_buf2_b, q_i2b);
    Convert_i2b_velocity(r_buf1_i, v_buf1_i, v_buf1_b, q_i2b, sc_body_rate);
    Convert_i2b_velocity(r_buf2_i, v_buf2_i, v_buf2_b, q_i2b, sc_body_rate);

    for (int j = 0; j < 3; j++) {
      celes_objects_pos_from_center_b_[i * 3 + j] = r_buf1_b[j];
      celes_objects_pos_from_sc_b_[i * 3 + j] = r_buf2_b[j];
      celes_objects_vel_from_center_b_[i * 3 + j] = v_buf1_b[j];
      celes_objects_vel_from_sc_b_[i * 3 + j] = v_buf2_b[j];
    }
  }
}

void Convert_i2b(const double* src_i, double* dst_b, Quaternion q_i2b) {
  Vector<3> temp_i;
  for (int i = 0; i < 3; i++) {
    temp_i[i] = src_i[i];
  }
  Vector<3> temp_b = q_i2b.frame_conv(temp_i);
  for (int i = 0; i < 3; i++) {
    dst_b[i] = temp_b[i];
  }
}

void Convert_i2b_velocity(const double* r_i, const double* v_i, double* v_b,
                          Quaternion q_i2b, const Vector<3> bodyrate_b) {
  // copy input vector
  Vector<3> vi;
  for (int i = 0; i < 3; i++) {
    vi[i] = v_i[i];
  }
  Vector<3> ri;
  for (int i = 0; i < 3; i++) {
    ri[i] = r_i[i];
  }

  // convert bodyrate vector into that in inertial coordinate
  Vector<3> wb;
  for (int i = 0; i < 3; i++) {
    wb[i] = bodyrate_b[i];
  }
  Vector<3> wi = q_i2b.frame_conv_inv(wb);
  // compute crossterm wxr
  Vector<3> wxr_i = outer_product(wb, ri);
  // compute dr/dt + wxr
  for (int i = 0; i < 3; i++) {
    vi[i] = vi[i] - wxr_i[i];
  }
  // convert vector in inertial coordinate into that in body coordinate
  Vector<3> temp_b = q_i2b.frame_conv(vi);
  for (int i = 0; i < 3; i++) {
    v_b[i] = temp_b[i];
  }
}

Vector<3> LocalCelestialInformation::GetPosFromSC_i(
    const char* body_name) const {
  Vector<3> position;
  int index = 0;
  index = glo_celes_info_->CalcBodyIdFromName(body_name);
  for (int i = 0; i < 3; i++) {
    position[i] = celes_objects_pos_from_sc_i_[index * 3 + i];
  }
  return position;
}

Vector<3> LocalCelestialInformation::GetPosFromSC_b(
    const char* body_name) const {
  Vector<3> position;
  int index = 0;
  index = glo_celes_info_->CalcBodyIdFromName(body_name);
  for (int i = 0; i < 3; i++) {
    position[i] = celes_objects_pos_from_sc_b_[index * 3 + i];
  }
  return position;
}

string LocalCelestialInformation::GetLogHeader() const {
  SpiceBoolean found;
  const int maxlen = 100;
  char namebuf[maxlen];
  string str_tmp = "";
  for (int i = 0; i < glo_celes_info_->GetNumBody(); i++) {
    SpiceInt planet_id = glo_celes_info_->GetSelectedBody()[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean*)&found);
    string name = namebuf;
    string body_pos = name + "_pos";
    string body_vel = name + "_vel";
    //　OUTPUT ONLY POS/VEL LOOKED FROM S/C AT THIS MOMENT
    str_tmp += WriteVector(body_pos, "b", "m", 3);
    str_tmp += WriteVector(body_vel, "b", "m/s", 3);
  }
  return str_tmp;
}

string LocalCelestialInformation::GetLogValue() const {
  string str_tmp = "";
  for (int i = 0; i < glo_celes_info_->GetNumBody(); i++) {
    //　OUTPUT ONLY POS/VEL LOOKED FROM S/C AT THIS MOMENT
    for (int j = 0; j < 3; j++) {
      str_tmp += WriteScalar(celes_objects_pos_from_sc_b_[i * 3 + j]);
    }
    for (int j = 0; j < 3; j++) {
      str_tmp += WriteScalar(celes_objects_vel_from_sc_b_[i * 3 + j]);
    }
  }
  return str_tmp;
}
