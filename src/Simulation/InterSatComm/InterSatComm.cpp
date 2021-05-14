#include "InterSatComm.h"
#include "Abstract/InterSatCommBase.h"

InterSatComm::InterSatComm()
{
}

InterSatComm::~InterSatComm()
{
}

void InterSatComm::RegisterCommElement(const std::string name_tag, InterSatCommBase* comm_element)
{
  comm_elements_.emplace(name_tag, comm_element);
}

void InterSatComm::RemoveCommElement(const std::string name_tag)
{
  comm_elements_.erase(name_tag);
}

InterSatCommBase* InterSatComm::GetCommElement(const std::string name_tag)
{
  return comm_elements_.at(name_tag);
}
