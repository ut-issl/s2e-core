#include "PowerPort.h"
#include <cfloat>

// Default constructor for users who don't want to use this feature
PowerPort::PowerPort()
  : kPortId(-1), 
    kCurrentLimit(10.0),
    kMinimumVoltage(3.3), 
    assumed_power_consumption_(0.0)
{
  is_on_ = true;  // power on to work the component
  Initialize();
}

PowerPort::PowerPort(int port_id, double current_Limit, double minimum_voltage, double assumed_power_consumption)
  : kPortId(port_id), 
    kCurrentLimit(current_Limit),
    kMinimumVoltage(minimum_voltage), 
    assumed_power_consumption_(assumed_power_consumption)
{
  Initialize();
}

PowerPort::~PowerPort()
{
}

void PowerPort::Initialize(void)
{
  voltage_ = 0.0f;
  current_consumption_ = 0.0f;
  assumed_power_consumption_ = 0.0f;
}

bool PowerPort::Update(void)
{
  // switching
  if(voltage_ >= (kMinimumVoltage-DBL_EPSILON))
  {
    is_on_ = true;
    current_consumption_ = assumed_power_consumption_/voltage_;
  }
  else
  {
    current_consumption_ = 0.0;
    is_on_ = false;
  }
  // over curret protection
  if(current_consumption_ >= (kCurrentLimit-DBL_EPSILON))
  {
    current_consumption_ = 0.0;
    voltage_ = 0.0;
    is_on_ = false;
  }
  return is_on_;
}

bool PowerPort::SetVoltage(const double voltage)
{
  voltage_ = voltage;
  Update();
  return is_on_;
}
