#include <iostream>
#include <stdio.h>
#include <string.h>
#include <tchar.h>

#include "src/Interface/HilsInOut/COSMOSWrapper.h"
#include "src/Interface/HilsInOut/HardwareMessage.h"
#include "src/Interface/InitInput/Initialize.h"
#include "src/Interface/LogOutput/Logger.h"
#include "src/Simulation/Case/EquuleusCase.h"
#include "src/Simulation/Case/SimulationCase.h"
#include "src/Simulation/MCSim/MCSimExecutor.h"

using namespace System::Threading::Tasks;

void mail_loop(int idx);

string ini_fname = "data/ini/SimBase.ini";

MCSimExecutor *mc_sim;

int _tmain_parallel(int argc, _TCHAR *argv[]) {
  mc_sim = InitMCSim("data/ini/SimBase.ini");
  Logger mclog("mont.csv");

  // Monte-Carlo Simulation
  // while (mc_sim->WillExecuteNextCase())
  //  concurrency::parallel_for(0, mc_sim->GetTotalNumOfExecutions(), //(auto
  //  i=0; i<mc_sim->GetTotalNumOfExecutions(); i++)
  Parallel::For(0, mc_sim->GetTotalNumOfExecutions(),
                gcnew System::Action<int>(mail_loop));
  return 0;
  //{
  //  SimulationCase& simcase = EquuleusCase(*mc_sim, ini_fname);
  //  mclog.AddLoggable(&simcase);
  //  if (mc_sim->GetNumOfExecutionsDone() == 0)
  //  {
  //    mclog.WriteHeaders();
  //  }
  //  simcase.Initialize();

  //  mclog.WriteValues();
  //  simcase.Main();
  //  mclog.WriteValues();
  //  mclog.ClearLoggables();
  //}
}

void mail_loop(int idx) {
  COSMOSWrapper &cosmos_wrapper = COSMOSWrapper();
  HardwareMessage &hw_msg = HardwareMessage();

  SimulationCase &simcase = EquuleusCase(
      *mc_sim, ini_fname, cosmos_wrapper,
      hw_msg); // 最終的にはnullptrではなくCOSMOSWrapperインスタンスへのポインタを渡すべき。
  // mclog.AddLoggable(&simcase);
  // if (mc_sim->GetNumOfExecutionsDone() == 0)
  //{
  //   mclog.WriteHeaders();
  // }
  simcase.Initialize();

  // mclog.WriteValues();
  simcase.Main();
  // mclog.WriteValues();
  // mclog.ClearLoggables();
}
