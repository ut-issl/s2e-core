#pragma once
#include <map>
#include <string>

class InterSatCommBase;

class InterSatComm
{
public:
  InterSatComm();
  ~InterSatComm();
  void RegisterCommElement(const std::string name_tag, InterSatCommBase* comm_element);
  void RemoveCommElement(const std::string name_tag);
  InterSatCommBase* GetCommElement(const std::string name_tag);

private:
  std::map<const std::string, InterSatCommBase*> comm_elements_;
};

