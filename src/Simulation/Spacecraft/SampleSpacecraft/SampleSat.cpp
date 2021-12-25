#include "SampleSat.h"
#include "SampleComponents.h"
#include "../../../Interface/InitInput/Initialize.h"
#include "../../../Dynamics/ClockGenerator.h"
#include "../../../Library/math/NormalRand.hpp"

SampleSat::SampleSat(SimulationConfig config)
  :Spacecraft(config)
{
  Initialize();
}

SampleSat::~SampleSat()
{
  delete components_;
  // この後親のデストラクタも勝手に呼ばれる
}

void SampleSat::Initialize()
{
  // 親のInitialize()はコンストラクタで呼ばれている
  components_ = new SampleComponents(dynamics_, &config_);
  IniAccess mainIni = IniAccess(config_.mainIniPath);
}

void SampleSat::LogSetup(Logger & logger)
{
  Spacecraft::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleSat::Update()
{
  // ダイナミクスのアップデート
  Spacecraft::Update(); // ここで軌道・姿勢伝搬 力はリセットされる
  // 模擬コンポの時を刻む
  for (int i = 0; i < config_.simTime->GetStepSec() * 1000; i++)
  {
    ClockGenerator::TickToComponents();
  }
}

void SampleSat::GenerateTorque_b()
{
  dynamics_->attitude_->AddTorque_b(components_->GenerateTorque_b());
}

void SampleSat::GenerateForce_b()
{
  dynamics_->orbit_->AddForce_b(components_->GenerateForce_b(), dynamics_->attitude_->GetQuaternion_i2b(), dynamics_->mass);
}


