#include "OBC_C2A.h"

#ifdef USE_C2A
  #include "command_analyze.h"
  #include "telemetry_frame.h"
  #include "packet_handler.h"
  #include "anomaly_logger.h"
  #include "app_manager.h"
  #include "app_registry.h"
  #include "time_manager.h"
  #include "block_command_table.h"
  #include "mode_manager.h"
  #include "task_dispatcher.h"
  #include "watchdog_timer.h"
#endif

std::map<int, SCIPort*>  OBC_C2A::com_ports_c2a_;
std::map<int, I2CPort*>  OBC_C2A::i2c_com_ports_c2a_;
std::map<int, GPIOPort*> OBC_C2A::gpio_ports_c2a_;

OBC_C2A::OBC_C2A(ClockGenerator* clock_gen)
:OBC(clock_gen), timing_regulator_(1)
{
  //Initialize();
}

OBC_C2A::OBC_C2A(ClockGenerator* clock_gen, int timing_regulator)
:OBC(clock_gen), timing_regulator_(timing_regulator)
{
  //Initialize();
}

OBC_C2A::OBC_C2A(int prescaler, ClockGenerator* clock_gen, int timing_regulator, PowerPort* power_port)
:OBC(prescaler, clock_gen, power_port), timing_regulator_(timing_regulator)
{
  //Initialize();
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
  if(is_initialized == false)
  {
    is_initialized = true;
    Initialize();
  }
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


int OBC_C2A::I2cConnectPort(int port_id, const unsigned char i2c_addr)
{
  if (i2c_com_ports_c2a_[port_id] != nullptr)
  {
    // Port already used
  }
  else
  {
    i2c_com_ports_c2a_[port_id] = new I2CPort();
  }
  i2c_com_ports_c2a_[port_id]->RegisterDevice(i2c_addr);
  
  return 0;
}

int OBC_C2A::I2cCloseComPort(int port_id)
{
  // Port not used
  if (i2c_com_ports_c2a_[port_id] == nullptr)  return -1;

  I2CPort *port = i2c_com_ports_c2a_.at(port_id);
  delete port;
  i2c_com_ports_c2a_.erase(port_id);
  return 0;
}

int OBC_C2A::I2cWriteRegister(int port_id, const unsigned char i2c_addr, const unsigned char* data, const unsigned char len)
{
  I2CPort* i2c_port = i2c_com_ports_c2a_[port_id];

  if(len == 1)
  {
    i2c_port->WriteRegister(i2c_addr, data[0]);
  }
  else
  {
    for(int i = 0; i < len-1; i++)
    {
      i2c_port->WriteRegister(i2c_addr, data[0]+i, data[i+1]);
    }
  }
  return 0;
}

int OBC_C2A::I2cReadRegister (int port_id, const unsigned char i2c_addr, unsigned char* data, const unsigned char len)
{
  I2CPort*  i2c_port = i2c_com_ports_c2a_[port_id];
  for(int i = 0; i < len; i++)
  {
    data[i] = i2c_port->ReadRegister(i2c_addr);
  }
  return 0;
}

int OBC_C2A::I2cComponentWriteRegister(int port_id, const unsigned char i2c_addr, const unsigned char reg_addr, const unsigned char* data, const unsigned char len)
{
  I2CPort* i2c_port = i2c_com_ports_c2a_[port_id];
  for(int i = 0; i < len; i++)
  {
    i2c_port->WriteRegister(i2c_addr, reg_addr+i, data[i]);
  }
  return 0;
}
int OBC_C2A::I2cComponentReadRegister (int port_id, const unsigned char i2c_addr, const unsigned char reg_addr, unsigned char* data, const unsigned char len)
{
  I2CPort*  i2c_port = i2c_com_ports_c2a_[port_id];
  for(int i = 0; i < len; i++)
  {
    data[i] = i2c_port->ReadRegister(i2c_addr, reg_addr+i);
  }
  return 0;
}

int OBC_C2A_I2cWriteRegister(int port_id, const unsigned char i2c_addr, const unsigned char* data, const unsigned char len)
{
  return OBC_C2A::I2cWriteRegister(port_id, i2c_addr, data, len);
}
int OBC_C2A_I2cReadRegister (int port_id, const unsigned char i2c_addr, unsigned char* data, const unsigned char len)
{
  return OBC_C2A::I2cReadRegister(port_id, i2c_addr, data, len);
}

int OBC_C2A::GpioConnectPort(int port_id)
{
  if (gpio_ports_c2a_[port_id] != nullptr)
  {
    // Port already used
    return -1;
  }
  gpio_ports_c2a_[port_id] = new GPIOPort(port_id);
  return 0;
}

int OBC_C2A::GpioComponentWrite(int port_id, const bool is_high)
{
  GPIOPort* port = gpio_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->DigitalWrite(is_high);
}

bool OBC_C2A::GpioComponentRead(int port_id)
{
  GPIOPort* port = gpio_ports_c2a_[port_id];
  if (port == nullptr) return false;
  return port->DigitalRead();
}

int  OBC_C2A::GpioWrite_C2A(int port_id, const bool is_high)
{
  GPIOPort* port = gpio_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->DigitalWrite(is_high);
}

bool OBC_C2A::GpioRead_C2A (int port_id)
{
  GPIOPort* port = gpio_ports_c2a_[port_id];
  if (port == nullptr) return false;
  return port->DigitalRead();
}


int OBC_C2A_GpioWrite(int port_id, const bool is_high)
{
  return OBC_C2A::GpioWrite_C2A(port_id, is_high);
}

bool OBC_C2A_GpioRead(int port_id)
{
  return OBC_C2A::GpioRead_C2A(port_id);
}