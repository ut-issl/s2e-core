#pragma once
class ITickable
{
public:
  virtual void Tick(int count) = 0;
};