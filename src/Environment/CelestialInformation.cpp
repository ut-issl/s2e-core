#include <iostream>
#include <sstream>

#include "CelestialInformation.h"
#include "../Interface/LogOutput/LogUtility.h"

using namespace std;

CelestialInformation::CelestialInformation(string inertial_frame, string aber_cor, string center_obj, int num_of_selected_body, int* selected_body) :
  num_of_selected_body_(num_of_selected_body)
{

  inertial_frame_ = inertial_frame;
  aber_cor_ = aber_cor;
  center_obj_ = center_obj;
  selected_body_ = selected_body;
  int num_of_state = num_of_selected_body_ * 3;

  celes_objects_pos_from_center_i_ = new double[num_of_state];
  celes_objects_pos_from_center_b_ = new double[num_of_state];
  celes_objects_pos_from_sc_i_ = new double[num_of_state];
  celes_objects_pos_from_sc_b_ = new double[num_of_state];
  celes_objects_vel_from_center_i_ = new double[num_of_state];
  celes_objects_vel_from_center_b_ = new double[num_of_state];
  celes_objects_vel_from_sc_i_ = new double[num_of_state];
  celes_objects_vel_from_sc_b_ = new double[num_of_state];
  celes_objects_gravity_constant_ = new double[num_of_selected_body_];
  // Acquisition of gravity constant
  for (int i = 0; i < num_of_selected_body_; i++)
  {
    SpiceInt planet_id = selected_body_[i];
    SpiceInt dim; SpiceDouble gravity_constant;
    bodvcd_c(planet_id, "GM", 1, &dim, &gravity_constant);
    // CONVERT FROM [km^3/s^2] to [m^3/s^2]
    celes_objects_gravity_constant_[i] = gravity_constant * 1E+9;
  }
}

CelestialInformation::CelestialInformation(const CelestialInformation & obj)
{
  num_of_selected_body_ = obj.num_of_selected_body_;
  inertial_frame_ = obj.inertial_frame_;
  aber_cor_ = obj.aber_cor_;
  center_obj_ = obj.center_obj_;
  int num_of_state = num_of_selected_body_ * 3;
  int sd = sizeof(double);
  int si = sizeof(int);

  selected_body_ = new int[num_of_selected_body_];
  celes_objects_pos_from_center_i_ = new double[num_of_state];
  celes_objects_pos_from_center_b_ = new double[num_of_state];
  celes_objects_pos_from_sc_i_ = new double[num_of_state];
  celes_objects_pos_from_sc_b_ = new double[num_of_state];
  celes_objects_vel_from_center_i_ = new double[num_of_state];
  celes_objects_vel_from_center_b_ = new double[num_of_state];
  celes_objects_vel_from_sc_i_ = new double[num_of_state];
  celes_objects_vel_from_sc_b_ = new double[num_of_state];
  celes_objects_gravity_constant_ = new double[num_of_selected_body_];

  memcpy(selected_body_, obj.selected_body_, si*num_of_selected_body_);
  memcpy(celes_objects_pos_from_center_i_, obj.celes_objects_pos_from_center_i_, sd*num_of_state);
  memcpy(celes_objects_pos_from_center_b_, obj.celes_objects_pos_from_center_b_, sd*num_of_state);
  memcpy(celes_objects_pos_from_sc_i_, obj.celes_objects_pos_from_sc_i_, sd*num_of_state);
  memcpy(celes_objects_pos_from_sc_b_, obj.celes_objects_pos_from_sc_b_, sd*num_of_state);
  memcpy(celes_objects_vel_from_center_i_, obj.celes_objects_vel_from_center_i_, sd*num_of_state);
  memcpy(celes_objects_vel_from_center_b_, obj.celes_objects_vel_from_center_b_, sd*num_of_state);
  memcpy(celes_objects_vel_from_sc_i_, obj.celes_objects_vel_from_sc_i_, sd*num_of_state);
  memcpy(celes_objects_vel_from_sc_b_, obj.celes_objects_vel_from_sc_b_, sd*num_of_state);
  memcpy(celes_objects_gravity_constant_, obj.celes_objects_gravity_constant_, sd*num_of_selected_body_);
}

CelestialInformation::~CelestialInformation()
{
  delete[] celes_objects_pos_from_center_i_;
  delete[] celes_objects_pos_from_center_b_;
  delete[] celes_objects_pos_from_sc_i_;
  delete[] celes_objects_pos_from_sc_b_;
  delete[] celes_objects_vel_from_center_i_;
  delete[] celes_objects_vel_from_center_b_;
  delete[] celes_objects_vel_from_sc_i_;
  delete[] celes_objects_vel_from_sc_b_;
  delete[] celes_objects_gravity_constant_;
  delete[] selected_body_;
}


void CelestialInformation::UpdateAllObjectsInfo(double current_jd, Vector<3> sc_pos_from_center_i, Vector<3> sc_vel_from_center_i, Quaternion q_i2b, Vector<3> sc_body_rate)
{
  // TODO: 宇宙機が複数になった場合への対応
  SpiceDouble rv_buf[6], lt, et;
  SpiceBoolean found;	const int maxlen = 100;	char namebuf[maxlen];
  double r_buf1_i[3], v_buf1_i[3], r_buf1_b[3], v_buf1_b[3];
  double r_buf2_i[3], v_buf2_i[3], r_buf2_b[3], v_buf2_b[3];
  string jd = "jd " + to_string(current_jd);
  str2et_c(jd.c_str(), &et);

  for (int i = 0; i < num_of_selected_body_; i++)
  {
    SpiceInt planet_id = selected_body_[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean*)&found);
    // Acquisition of position and velocity
    spkezr_c(namebuf, et, inertial_frame_.c_str(), aber_cor_.c_str(), center_obj_.c_str(), (SpiceDouble*)rv_buf, (SpiceDouble*)&lt);
    // CONVERT [km], [km/s] to [m], [m/s] about Body info from SPICE
    for (int j = 0; j < 6; j++) { rv_buf[j] = rv_buf[j] * 1000; }
    for (int j = 0; j < 3; j++)
    {
      celes_objects_pos_from_center_i_[i * 3 + j] = rv_buf[j];
      celes_objects_pos_from_sc_i_[i * 3 + j] = rv_buf[j] - sc_pos_from_center_i[j];
      celes_objects_vel_from_center_i_[i * 3 + j] = rv_buf[j + 3];
      celes_objects_vel_from_sc_i_[i * 3 + j] = rv_buf[j + 3] - sc_vel_from_center_i[j];

      r_buf1_i[j] = rv_buf[j];
      r_buf2_i[j] = rv_buf[j] - sc_pos_from_center_i[j];
      v_buf1_i[j] = rv_buf[j + 3];
      v_buf2_i[j] = rv_buf[j + 3] - sc_vel_from_center_i[j];
    }
    Convert_i2b(r_buf1_i, r_buf1_b, q_i2b);
    Convert_i2b(r_buf2_i, r_buf2_b, q_i2b);
    Convert_i2b_velocity(r_buf1_i, v_buf1_i, v_buf1_b, q_i2b, sc_body_rate);
    Convert_i2b_velocity(r_buf2_i, v_buf2_i, v_buf2_b, q_i2b, sc_body_rate);

    for (int j = 0; j < 3; j++)
    {
      celes_objects_pos_from_center_b_[i * 3 + j] = r_buf1_b[j];
      celes_objects_pos_from_sc_b_[i * 3 + j] = r_buf2_b[j];
      celes_objects_vel_from_center_b_[i * 3 + j] = v_buf1_b[j];
      celes_objects_vel_from_sc_b_[i * 3 + j] = v_buf2_b[j];
    }
  }
}


void CelestialInformation::CalcAllPosVel_b(Quaternion q_i2b, Vector<3> sc_body_rate)
{
  double r_buf1_i[3], v_buf1_i[3], r_buf1_b[3], v_buf1_b[3];
  double r_buf2_i[3], v_buf2_i[3], r_buf2_b[3], v_buf2_b[3];
  for (int i = 0; i < num_of_selected_body_; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      r_buf1_i[j] = celes_objects_pos_from_center_i_[i * 3 + j];
      r_buf2_i[j] = celes_objects_pos_from_sc_i_[i * 3 + j];
      v_buf1_i[j] = celes_objects_vel_from_center_i_[i * 3 + j];
      v_buf2_i[j] = celes_objects_vel_from_sc_i_[i * 3 + j];
    }
    Convert_i2b(r_buf1_i, r_buf1_b, q_i2b);
    Convert_i2b(r_buf2_i, r_buf2_b, q_i2b);
    Convert_i2b_velocity(r_buf1_i, v_buf1_i, v_buf1_b, q_i2b, sc_body_rate);
    Convert_i2b_velocity(r_buf2_i, v_buf2_i, v_buf2_b, q_i2b, sc_body_rate);

    for (int j = 0; j < 3; j++)
    {
      celes_objects_pos_from_center_b_[i * 3 + j] = r_buf1_b[j];
      celes_objects_pos_from_sc_b_[i * 3 + j] = r_buf2_b[j];
      celes_objects_vel_from_center_b_[i * 3 + j] = v_buf1_b[j];
      celes_objects_vel_from_sc_b_[i * 3 + j] = v_buf2_b[j];
    }
  }
}

void Convert_i2b(const double* src_i, double* dst_b, Quaternion q_i2b)
{
  Vector<3> temp_i;
  for (int i = 0; i < 3; i++) { temp_i[i] = src_i[i]; }
  Vector<3> temp_b = q_i2b.frame_conv(temp_i);
  for (int i = 0; i < 3; i++) { dst_b[i] = temp_b[i]; }
}

void Convert_i2b_velocity(const double* r_i, const double* v_i, double* v_b, Quaternion q_i2b, const Vector<3> bodyrate_b)
{
  // copy input vector
  Vector<3> vi;
  for (int i = 0; i < 3; i++) { vi[i] = v_i[i]; }
  Vector<3> ri;
  for (int i = 0; i < 3; i++) { ri[i] = r_i[i]; }

  // convert bodyrate vector into that in inertial coordinate
  Vector<3> wb;
  for (int i = 0; i < 3; i++) { wb[i] = bodyrate_b[i]; }
  Vector<3> wi = q_i2b.frame_conv_inv(wb);
  // compute crossterm wxr
  Vector<3> wxr_i = outer_product(wb, ri);
  // compute dr/dt + wxr
  for (int i = 0; i < 3; i++) { vi[i] = vi[i] - wxr_i[i]; }
  // convert vector in inertial coordinate into that in body coordinate
  Vector<3> temp_b = q_i2b.frame_conv(vi);
  for (int i = 0; i < 3; i++) { v_b[i] = temp_b[i]; }
}

Vector<3> CelestialInformation::GetPosFromSC_i(const char* body_name) const
{
  Vector<3> position; int index = 0;
  SpiceInt planet_id;	SpiceBoolean found;	const int maxlen = 100;
  // Acquisition of ID from body name
  bodn2c_c(body_name, (SpiceInt*)&planet_id, (SpiceBoolean*)&found);
  for (int i = 0; i < num_of_selected_body_; i++)
  {
    if (selected_body_[i] == planet_id) { index = i; break; }
  }
  for (int i = 0; i < 3; i++) { position[i] = celes_objects_pos_from_sc_i_[index * 3 + i]; }
  return position;
}

Vector<3> CelestialInformation::GetPosFromSC_b(const char* body_name) const
{
  Vector<3> position; int index = 0;
  SpiceInt planet_id;	SpiceBoolean found;	const int maxlen = 100;
  // Acquisition of ID from body name
  bodn2c_c(body_name, (SpiceInt*)&planet_id, (SpiceBoolean*)&found);
  for (int i = 0; i < num_of_selected_body_; i++)
  {
    if (selected_body_[i] == planet_id) { index = i; break; }
  }
  for (int i = 0; i < 3; i++)
  {
    position[i] = celes_objects_pos_from_sc_b_[index * 3 + i];
  }
  return position;
}

Vector<6> CelestialInformation::FetchPosVel(int id, double current_jd)
{
  Vector<6> posvel;
  SpiceBoolean found;	const int maxlen = 100;	char namebuf[maxlen];

  SpiceInt planet_id = id;
  SpiceDouble rv_buf[6], lt, et;
  string jd = "jd " + to_string(current_jd);
  str2et_c(jd.c_str(), &et);
  // Acquisition of body name from id
  bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean*)&found);
  // Acquisition of position and velocity
  spkezr_c(namebuf, et, inertial_frame_.c_str(), aber_cor_.c_str(), center_obj_.c_str(), (SpiceDouble*)rv_buf, (SpiceDouble*)&lt);
  // CONVERT [km], [mkm/s] to [m], [m/s] about Body info from SPICE
  for (int i = 0; i < 6; i++) { posvel[i] = rv_buf[i] * 1000; }
  return posvel;
}

Vector<6> CelestialInformation::FetchPosVel(char* name, double current_jd)
{
  Vector<6> posvel;
  const int maxlen = 100;

  SpiceDouble rv_buf[6], lt, et;
  string jd = "jd " + to_string(current_jd);
  str2et_c(jd.c_str(), &et);
  // Acquisition of position and velocity
  spkezr_c(name, et, inertial_frame_.c_str(), aber_cor_.c_str(), center_obj_.c_str(), (SpiceDouble*)rv_buf, (SpiceDouble*)&lt);
  // CONVERT [km], [mkm/s] to [m], [m/s] about Body info from SPICE
  for (int i = 0; i < 6; i++) { posvel[i] = rv_buf[i] * 1000; }
  return posvel;
}

double CelestialInformation::GetGravityConstant(const char* body_name)
{
  Vector<3> position; int index = 0;
  SpiceInt planet_id;	SpiceBoolean found;	const int maxlen = 100;
  // Acquisition of ID from body name
  bodn2c_c(body_name, (SpiceInt*)&planet_id, (SpiceBoolean*)&found);
  for (int i = 0; i < num_of_selected_body_; i++)
  {
    if (selected_body_[i] == planet_id) { index = i; break; }
  }
  return celes_objects_gravity_constant_[index];
}

string CelestialInformation::GetLogHeader() const
{
  SpiceBoolean found;	const int maxlen = 100;	char namebuf[maxlen];
  string str_tmp = "";
  for (int i = 0; i < num_of_selected_body_; i++)
  {
    SpiceInt planet_id = selected_body_[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean*)&found);
    string name = namebuf;
    string body_pos = name + "_pos";
    string body_vel = name + "_vel";
    //　OUTPUT ONLY POS/VEL LOOKED FROM S/C AT THIS MOMENT
    str_tmp += WriteVector(body_pos, "i", "m", 3);
    str_tmp += WriteVector(body_vel, "i", "m/s", 3);
    str_tmp += WriteVector(body_pos, "b", "m", 3);
    str_tmp += WriteVector(body_vel, "b", "m/s", 3);
  }
  return str_tmp;
}

string CelestialInformation::GetLogValue() const
{
  string str_tmp = "";
  for (int i = 0; i < num_of_selected_body_; i++)
  {
    //　OUTPUT ONLY POS/VEL LOOKED FROM S/C AT THIS MOMENT
    for (int j = 0; j < 3; j++) { str_tmp += WriteScalar(celes_objects_pos_from_center_i_[i * 3 + j]); }
    for (int j = 0; j < 3; j++) { str_tmp += WriteScalar(celes_objects_vel_from_center_i_[i * 3 + j]); }
    for (int j = 0; j < 3; j++) { str_tmp += WriteScalar(celes_objects_pos_from_sc_b_[i * 3 + j]); }
    for (int j = 0; j < 3; j++) { str_tmp += WriteScalar(celes_objects_vel_from_sc_b_[i * 3 + j]); }

  }
  return str_tmp;
}

void CelestialInformation::DebugOutput(void)
{
  SpiceBoolean found;	const int maxlen = 100;	char namebuf[maxlen];
  cout << "BODY NAME, POSx,y,z[m], VELx,y,z[m/s] from CENTER;\nPOSx,y,z[m], VELx,y,z[m/s] from SC";
  for (int i = 0; i < num_of_selected_body_; i++)
  {
    SpiceInt planet_id = selected_body_[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean*)&found);
    //		cout<<namebuf<<
  }
  cout << "GRAVITY CONSTASNT of\n";
  for (int i = 0; i < num_of_selected_body_; i++)
  {
    SpiceInt planet_id = selected_body_[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean*)&found);
    cout << namebuf << "is" << ": " << celes_objects_gravity_constant_[i] << "\n";
  }
}

