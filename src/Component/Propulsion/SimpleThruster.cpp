#include <math.h>

#include "SimpleThruster.h"
#include "../../Library/math/GlobalRand.h"

#define DEG2RAD 0.017453292519943295769  // PI/180

//コンストラクタ
SimpleThruster::SimpleThruster(ClockGenerator* clock_gen, Vector<3> thruster_pos, Vector<3> thrust_dir, double max_mag, double mag_err, double deg_err, int id, const Structure* structure, const Dynamics* dynamics)
  :ComponentBase(1000,clock_gen),mag_nr(0, mag_err, id), dir_nr(0, deg_err, id), id_(id), structure_(structure),dynamics_(dynamics)
{
  thruster_pos_ = thruster_pos;//スラスタ位置ベクトル格納
  thrust_dir_ = normalize(thrust_dir);//スラスト方向ベクトル正規化
  thrust_magnitude_max = max_mag; // スラスト最大値
  thrust_magnitude_err_ = mag_err;//スラスト誤差大きさ設定,正規分布関数の標準偏差を入力
  thrust_dir_err_ = deg_err;//誤差角の大きさ設定、正規分布関数の標準偏差を入力
  thrust_b_ *= 0;
  duty_ = 0;
}

SimpleThruster::~SimpleThruster(){
}

void SimpleThruster::MainRoutine(int count){
  SetDuty(0); //閉めておく
  CalcThrust();
  CalcTorque(structure_->GetKinematicsParams().GetCGb(),0);
}

void SimpleThruster::SetDuty(double dutyratio)
{
  duty_ = dutyratio;
}

//並進力計算
void SimpleThruster::CalcThrust(bool isReal)
{
  double mag = CalcThrustMagnitude();
  if (isReal && duty_ != 0) mag += mag_nr;
  thrust_b_ = mag * CalcThrustDir();
}

//トルク計算
void SimpleThruster::CalcTorque(Vector<3> center, double temp)
{

  Vector<3> vector_center2thruster = thruster_pos_ - center;//重心位置計算
  Vector<3> torque = outer_product(vector_center2thruster, GetThrust());//トルク計算(外積)

  torque_b_ = torque;
}

Vector<3> SimpleThruster::GetThrust()
{
  return thrust_b_;
}

Vector<3> SimpleThruster::GetTorque()
{
  return torque_b_;
}

string SimpleThruster::GetLogHeader() const
{
  string str_tmp = "";

  string head = "TH" + to_string(id_);
  str_tmp += WriteVector(head+"thrust", "b", "N", 3);
  str_tmp += WriteVector(head+"torque", "b", "Nm", 3);
  str_tmp += WriteScalar(head + "thrust", "N");
  return str_tmp;
}

string SimpleThruster::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteVector(thrust_b_);
  str_tmp += WriteVector(torque_b_);
  str_tmp += WriteScalar(norm(thrust_b_));

  return str_tmp;
}

//バルブの数による推力の大きさ絶対値計算、温度が必要
double SimpleThruster::CalcThrustMagnitude()
{
  //バルブが開いたらスラスト計算(バルブが閉じている場合はゼロ．)
  return duty_* thrust_magnitude_max;
}

//方向の誤差追加(正規化方向ベクトルに対してクオータニオン処理を行う。INITファイルでの誤差値を0にすれば、誤差は出ずにそのまま返される)
//回転軸ベクトルは、誤差がない場合のベクトルに垂直なベクトルの中からランダムに選ぶので良い
//!!!外積で直行ベクトルを出してから角度を変更したら良い
//!!!もしこれで行くなら、分母が0のときで場合分けしないと、あとはランダム値は正規分布を実装
//!!!randは正の値しか出ない
Vector<3> SimpleThruster::CalcThrustDir()
{

  Vector<3> thrust_dir_true = thrust_dir_;//正規化方向ベクトル(誤差なし)
  if (thrust_dir_err_ != 0)
  {
    Vector<3> ez;
    ez[0] = 0;
    ez[1] = 0;
    ez[2] = 1;
    Vector<3> ex;//スラスタベクトルに垂直なベクトルの一例
    ex[0] = 1;
    ex[1] = 0;
    ex[2] = 0;


    double flag = rand() % 2;
    double make_axis_rot_deg;
    if (flag == 0)
    {
      make_axis_rot_deg = rand();
    }
    if (flag == 1)
    {
      make_axis_rot_deg = -rand();
    }

    Quaternion make_axis_rot(thrust_dir_true, make_axis_rot_deg*DEG2RAD);//回転軸を回す角度はとりあえずランダムな正負の整数値にしてある
    Vector<3> axis_rot = make_axis_rot.frame_conv(ex);//x軸をスラスタ初期ベクトルを軸に回して、ランダムな回転軸を生成
    NormalRand make_dir_err_deg(0, thrust_dir_err_*DEG2RAD,  g_rand.MakeSeed());//正規分布に従った角度誤差を作るオブジェクト、平均は0°標準偏差に入力された角度誤差
    Quaternion err_rot(axis_rot, make_dir_err_deg.operator double());//誤差を与えるクオータニオン生成、角度誤差から毎回正規分布に従った角度誤差をランダムに出し続ける
    thrust_dir_true = err_rot.frame_conv(thrust_dir_true);//クオータニオンによる誤差追加
  }
  /*
    if(thrust_dir_err_ != 0){
    Vector<3> axis_rot;
    srand((unsigned int)time(NULL));
    axis_rot[0] = rand();
    axis_rot[1] = rand();
    //ここ合ってる？axis_rot[2] = thruster_pos_[2] - thrust_dir_true[0] / thrust_dir_true[2] * axis_rot[0] - thrust_dir_true[1] / thrust_dir_true[2] * axis_rot[1];//回転軸、誤差ない時の推力べクトルに垂直なベクトル
    axis_rot[2] = thruster_pos_[2] - thrust_dir_true[0] / thrust_dir_true[2] * (axis_rot[0] - thruster_pos_[0]) - thrust_dir_true[1] / thrust_dir_true[2] * (axis_rot[1] - thruster_pos_[1]);//回転軸、誤差ない時の推力べクトルに垂直なベクトル
        Quaternion err_rot(axis_rot,-thrust_dir_err_);//誤差を与えるクオータニオン生成
        err_rot.frame_conv(thrust_dir_true);//クオータニオンによる誤差追加
    }
  */
  return thrust_dir_true;
}
