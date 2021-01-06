#include "OBC_C2A.h"

#ifdef USE_C2A
  #include "CommandAnalyze.h"
  #include "TelemetryFrame.h"
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

std::map<int, SCIPort*> OBC_C2A::com_ports_c2a_;

OBC_C2A::OBC_C2A(ClockGenerator* clock_gen)
:OBC(clock_gen), timing_regulator_(1)
{
  Initialize();
}

OBC_C2A::OBC_C2A(ClockGenerator* clock_gen, int timing_regulator)
:OBC(clock_gen), timing_regulator_(timing_regulator)
{
  Initialize();
}

OBC_C2A::OBC_C2A(int prescaler, ClockGenerator* clock_gen, double current, int timing_regulator)
:OBC(prescaler, clock_gen, current), timing_regulator_(timing_regulator)
{
  Initialize();
}

OBC_C2A::~OBC_C2A()
{
}

void OBC_C2A::Initialize()
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
void OBC_C2A::MainRoutine(int count)
{
#ifdef USE_C2A
  for(int i=0;i<timing_regulator_;i++)
  {
    TMGR_count_up_master_clock();   //The update time oc C2A clock should be 1msec
    TDSP_execute_pl_as_task_list();
  }
#endif
}

// Override functions
int OBC_C2A::ConnectComPort(int port_id, int tx_buf_size, int rx_buf_size)
{
  if (com_ports_c2a_[port_id] != nullptr)
  {
    // Port already used
    return -1;
  }
  com_ports_c2a_[port_id] = new SCIPort(tx_buf_size, rx_buf_size);
  return 0;
}

// Close port and free resources
int OBC_C2A::CloseComPort(int port_id)
{
  // Port not used
  if (com_ports_c2a_[port_id] == nullptr)
    return -1;

  SCIPort *port = com_ports_c2a_.at(port_id);
  delete port;
  com_ports_c2a_.erase(port_id);
  return 0;
}

int OBC_C2A::SendFromObc(int port_id, unsigned char* buffer, int offset, int count)
{
  return OBC_C2A::SendFromObc_C2A(port_id, buffer, offset, count);
}

int OBC_C2A::ReceivedByCompo(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = com_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->ReadTx(buffer, offset, count);
}

int OBC_C2A::SendFromCompo(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = com_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->WriteRx(buffer, offset, count);
}

int OBC_C2A::ReceivedByObc(int port_id, unsigned char* buffer, int offset, int count)
{
  return OBC_C2A::ReceivedByObc_C2A(port_id, buffer, offset, count);
}

// Static functions
int OBC_C2A::SendFromObc_C2A(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = com_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->WriteTx(buffer, offset, count);
}
int OBC_C2A::ReceivedByObc_C2A(int port_id, unsigned char* buffer, int offset, int count)
{
  SCIPort* port = com_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->ReadRx(buffer, offset, count);
}

// If the character encoding of C2A is UTF-8, these functions are not necessary, and users can directory use SendFromObc_C2A and ReceivedByObc_C2A
int OBC_C2A_SendFromObc(int port_id, unsigned char* buffer, int offset, int count)
{
  return OBC_C2A::SendFromObc_C2A(port_id, buffer, offset, count);
}
int OBC_C2A_ReceivedByObc(int port_id, unsigned char* buffer, int offset, int count)
{
  return OBC_C2A::ReceivedByObc_C2A(port_id, buffer, offset, count);
}