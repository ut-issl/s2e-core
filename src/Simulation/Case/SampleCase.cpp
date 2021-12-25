#include "SampleCase.h"

#include "../../Interface/InitInput/Initialize.h"
#include "../SimulationConfig.h"

#include "../../Disturbance/Disturbances.h"
#include "../../Environment/Environment.h"

#include "../Spacecraft/SampleSpacecraft/SampleSat.h"

//TODO: Refactor to improve base class
SampleCase::SampleCase(string ini_fname, string data_path)
  : ini_fname(ini_fname), data_path(data_path)
{
}

SampleCase::~SampleCase()
{
  delete environment;
  delete disturbances;
  delete sample_sat;
  delete sim_time;
  delete default_log;
}

void SampleCase::Initialize()
{
  //シミュレーション条件の設定
  //シミュレーション時間
  sim_time      = InitSimTime(ini_fname);
  //ログファイル名
  default_log = InitLog(ini_fname);
  //config
  SimulationConfig config = {ini_fname, sim_time, default_log};

  //シミュレーションされるもののインスタンス化
  environment  = new Envir(ini_fname);
  disturbances = new Disturbances(ini_fname);
  sample_sat = new SampleSat(config);

  //ログ出力の登録
  default_log->AddLoggable(sim_time);
  sample_sat->LogSetup(*default_log);
  environment->LogSetup(*default_log);
  disturbances->LogSetup(*default_log);

  //ログにヘッダを書き込み
  default_log->WriteHeaders();

  //シミュレーション開始
  cout << "\nSimulationDateTime \n";
  sim_time->PrintStartDateTime();
}

void SampleCase::Main()
{
  sim_time->ResetClock();
  while (!sim_time->GetState().finish)
  {
    //ログ書き出し
    if (sim_time->GetState().log_output)
    {
      default_log->WriteValues();
    }

    // Time update
    sim_time->UpdateTime();
    //環境と加わる外乱の更新
    environment->Update(*sample_sat, *sim_time);
    disturbances->Update(*environment, *sample_sat);

    //次のダイナミクス伝搬のため外乱で発生した力・トルクを加える
    sample_sat->dynamics_->AddAcceleration_i(disturbances->GetAccelerationI());
    sample_sat->dynamics_->AddTorque_b(disturbances->GetTorque());
    sample_sat->dynamics_->AddForce_b(disturbances->GetForce());
    //次のダイナミクス伝搬のため衛星が発生させる力・トルクを加える
    sample_sat->GenerateTorque_b();
    //sample_sat->GenerateForce_b();

    //衛星内のダイナミクス，コンポーネントの更新
    sample_sat->Update();


    if (sim_time->GetState().disp_output)
    {
      cout << "Progresss: " << sim_time->GetProgressionRate() << "%\r";
    }
  }
}

string SampleCase::GetLogHeader() const
{
  string str_tmp = "";

  str_tmp += WriteScalar("time", "s");
  str_tmp += WriteVector("position", "i", "m", 3);
  str_tmp += WriteVector("velocity", "i", "m/s", 3);
  str_tmp += WriteVector("quaternion", "i2b", "-", 4);
  str_tmp += WriteVector("omega", "b", "-", 3);

  return str_tmp;
}

string SampleCase::GetLogValue() const
{
  string str_tmp = "";

  auto pos_i = sample_sat->dynamics_->GetOrbit().GetSatPosition_i();
  auto vel_i = sample_sat->dynamics_->GetOrbit().GetSatVelocity_i();
  auto quat_i2b = sample_sat->dynamics_->GetAttitude().GetQuaternion_i2b();
  auto omega_b = sample_sat->dynamics_->GetAttitude().GetOmega_b();
  

  //＊上のヘッダと内容を一致させる
  str_tmp += WriteScalar(sim_time->GetElapsedSec());
  str_tmp += WriteVector(pos_i, 16);
  str_tmp += WriteVector(vel_i, 10);
  str_tmp += WriteQuaternion(quat_i2b);
  str_tmp += WriteVector(omega_b, 10);

  return str_tmp;
}
