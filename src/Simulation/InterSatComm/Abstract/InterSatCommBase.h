#pragma once
#include <string>

class InterSatComm;

class InterSatCommBase
{
public:
  InterSatCommBase(const std::string name_tag, InterSatComm* inter_sat_comm);
  InterSatCommBase(const InterSatCommBase& obj);
  ~InterSatCommBase();

private:
  InterSatComm* inter_sat_comm_;
  const std::string name_tag_;
};

