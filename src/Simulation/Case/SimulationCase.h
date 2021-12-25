#pragma once
#include "../../Interface/LogOutput/ILoggable.h"

class SimulationCase : public ILoggable
{
public:
  virtual ~SimulationCase() {}
  virtual void Initialize() = 0;
  //virtual void Run1Step();
  virtual void Main() = 0;

  virtual string GetLogHeader() const = 0;
  virtual string GetLogValue() const = 0;
};


