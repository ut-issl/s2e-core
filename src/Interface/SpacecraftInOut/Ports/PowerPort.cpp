#include "PowerPort.h"
#include "../../../Component/Abstract/ComponentBase.h"


PowerPort::PowerPort(int port_id, double currentLimit, ComponentBase* component)
  : kPortId(port_id), kCurrentLimit(currentLimit), component_(component)
{
  SetVoltage(-1);
}

PowerPort::~PowerPort()
{
}

double PowerPort::GetCurrent()
{
  double current = component_->GetCurrent(kPortId);
  if (current > kCurrentLimit)
  {
    // オーバーカレントプロテクション！！！
    SetVoltage(0);
    return 0;
  }
  return current;
}

double PowerPort::GetVoltage()
{
  return voltage_;
}

void PowerPort::SetVoltage(double voltage)
{
  // 電圧に変化があったら、コンポに知らせる
  // 電圧変化に対する反応は、コンポ側の実装に任せる
  if (voltage_ == voltage) return;
  voltage_ = voltage;
  component_->SetPowerState(kPortId, voltage_);
}

ComponentBase * PowerPort::GetComponent()
{
  return component_;
}

void PowerPort::SetComponent(ComponentBase * component)
{
  component_ = component;
}
