#include "OBC.h"

#ifdef USE_C2A
  #include "cmd_analyze_mobc.h"
  #include "tlm_frame_mobc.h"
  #include "PacketHandler.h"
  #include "AnomalyLogger.h"
  #include "AppManager.h"
  #include "AppRegistry.h"
  #include "TimeManager.h"
  #include "BlockCommandTable.h"
  #include "ModeManager.h"
  #include "TaskDispatcher.h"
  #include "WatchdogTimer.h"
#endif
#include <iostream>
OBC::OBC(ClockGenerator* clock_gen) : ComponentBase(1,clock_gen)
{
  Initialize();
}

OBC::~OBC()
{
}

void OBC::Initialize()
{
#ifdef USE_C2A
  CA_initialize();            //Cmd Analyze
  TF_initialize();            //TLM frame
  PH_init();                  //Packet Handler
  TMGR_init();                //Time Manager
  AL_initialize();            //Anomaly Logger
  AM_initialize();            //App Manager
  AR_load_initial_settings();	//App Registry
  AM_initialize_all_apps();	  //App Managerに登録されてるアプリの初期化
  BCT_initialize();	          //Block Cmd Table
  MM_initialize();            //Mode Manager
  TDSP_initialize();          //Task Dispatcher
  WDT_init();                 // WDT
#endif
}

double OBC::GetCurrent(int port_id) const
{
  if (isOn_)
  {
    return 0.1;
  }
  return 0;
}

void OBC::MainRoutine(int count)
{
#ifdef USE_C2A
  #ifdef C2A_EQUULEUS
    TMGR_count_up();
  #else
    TMGR_count_up_master_clock();
  #endif
  TDSP_execute_pl_as_task_list();
#endif
}
