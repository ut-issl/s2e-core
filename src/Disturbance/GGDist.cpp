#include "GGDist.h"

#include <cmath>
#include <fstream>
#include <iostream>

#include "../Interface/LogOutput/LogUtility.h"

using namespace std;

GGDist::GGDist() : GGDist(3.986004418 * pow(10.0, 14.0)) {  //デフォルトコンストラクタ
}

GGDist::GGDist(const double mu_e_input) {  //コンストラクタ
  fill_up(torque_b_, 0.0);
  mu_e_ = mu_e_input;
  kilo_ = 1000.0;
}

void GGDist::Update(const LocalEnvironment& local_env, const Dynamics& dynamics) {
  CalcTorque(local_env.GetCelesInfo().GetPosFromSC_b("EARTH"), dynamics.GetAttitude().GetInertiaTensor());
}

Vector<3> GGDist::CalcTorque(
    double R0, Vector<3> u_b,
    Matrix<3, 3> I_b) {  //トルクを取得、引数は前から地球半径、地球中心方向単位ベクトル、慣性テンソル。引数の単位はそれぞれm,無次元,kg*m^2
  torque_b_ = 3.0 * mu_e_ / pow(R0, 3.0) * outer_product(normalize(u_b), I_b * normalize(u_b));
  return torque_b_;
}

Vector<3> GGDist::CalcTorque(
    Vector<3> r_b, Matrix<3, 3> I_b) {  //トルクを取得、引数は前から地球中心方向ベクトル(大きさ付き)、慣性テンソル。引数の単位はそれぞれm,kg*m^2
  double coeff = 3.0 * mu_e_ / pow(norm(r_b), 3.0);
  torque_b_ = coeff * outer_product(normalize(r_b), I_b * normalize(r_b));
  return torque_b_;
}

string GGDist::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("ggtorque", "b", "Nm", 3);

  return str_tmp;
}

string GGDist::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(torque_b_);

  return str_tmp;
}

/*int main(){
    GGDist gg;
    Vector<3> u;
    u[0] = 0.8;
    u[1] = 0.48;
    u[2] = 0.36;
    Matrix<3,3> I(0.0);
    I[0][0] = 10;
    I[1][1] = 15;
    I[2][2] = 21;
    ofstream log;//ファイル書き込み用
    log.open("log.csv",ios::trunc);
    ofstream log1;//ファイル書き込み用
    log.open("log1.csv",ios::trunc);
    for(int i = 6400;i < 10000;i++){
        double R = double(i);
        Vector<3> r = R * u;
        log << gg.GetTorque(R,u,I)[0] << "," << gg.GetTorque(R,u,I)[1] << "," <<
gg.GetTorque(R,u,I)[2] << endl;
    }
}*/
