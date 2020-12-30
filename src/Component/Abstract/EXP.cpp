#include "EXP.h"
#include <string.h>
#include "../../Interface/SpacecraftInOut/GPIODriver.h"

EXP::EXP(ClockGenerator* clock_gen, int port_id, OBC* obc) : ComponentBase(1000,clock_gen), obc_(obc)
{
  port_id_ = port_id;
  obc_->ConnectComPort(port_id_,1024,1024);
  Initialize();
}

int EXP::Initialize()
{
  for (int i = 0; i < MAX_MEMORY_LEN; i++)
  {
    memory.push_back(0);
  }
  return 0;
}

EXP::~EXP()
{
  obc_->CloseComPort(port_id_);
}

int EXP::ReceiveCommand()
{
  unsigned char rxb[5]={0,0,0,0,0};
  int ret = obc_->ReceivedByCompo(port_id_, rxb, 0, 5);
  if (ret == 0) return 0;
  for (int i = 0; i < MAX_MEMORY_LEN; i++)
  {
    rx_buff[i] = rxb[i];
  }
  ParseCommand(rx_buff);

  return 0;
}

int EXP::ParseCommand(unsigned char * cmd)
{
  if (sizeof(cmd) < 4)
  {
    return -1;
  }
  if (cmd[0] != 'S' || cmd[1] != 'E' || cmd[2] != 'T')
  {
    return -1;
  }
  memory.pop_back();
  memory.insert(memory.begin(), cmd[3]);
  memory[MAX_MEMORY_LEN - 1] = '\n';
  return 0;
}

int EXP::SendTelemetry()
{
  for (int i = 0; i < MAX_MEMORY_LEN; i++)
  {
    tx_buff[i] = (unsigned char)memory[i];
  }
  obc_->SendFromCompo(port_id_, tx_buff, 0, MAX_MEMORY_LEN+1);
  return 0;
}

void EXP::MainRoutine(int count)
{
  ReceiveCommand();
  SendTelemetry();
}

void EXP::GPIOStateChanged(int port_id, bool isPosedge)
{
  printf("interrupted");
}

double EXP::GetCurrent(int port_id) const
{
  if (!isOn_) return 0;
  return 0.5;
}
