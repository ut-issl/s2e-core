#pragma once


#include "../../Interface/InitInput/Initialize.h"
#include "../SimulationConfig.h"
#include "../../Dynamics/Dynamics.h"
#include "../../Environment/Environment.h"


class Spacecraft
{
public:
  Spacecraft(SimulationConfig config);
  Spacecraft(SimulationConfig config, string AttitudeName, string OrbitName);
  virtual ~Spacecraft();

  // ちょっとコピー周りの対応がめんどいのでとりあえずコピー禁止しとく
  Spacecraft(const Spacecraft &) = delete;
  Spacecraft& operator= (const Spacecraft &) = delete;

  // 初期化
  virtual void Initialize(string AttitudeName="ATTITUDE", string OrbitName="ORBIT");

  virtual void LogSetup(Logger& logger);

  // 状態量の更新
  virtual void Update();

  Dynamics* dynamics_;
  Envir* environments_;


protected:

  const SimulationConfig config_;
};

