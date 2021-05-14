#include "InterSatCommBase.h"
#include "../InterSatComm.h"

InterSatCommBase::InterSatCommBase(const std:: string name_tag, InterSatComm* inter_sat_comm) : name_tag_(name_tag), inter_sat_comm_(inter_sat_comm)
{
  inter_sat_comm_->RegisterCommElement(name_tag, this);
}

InterSatCommBase::InterSatCommBase(const InterSatCommBase& obj)
{
  inter_sat_comm_->RegisterCommElement(obj.name_tag_, this);
}

InterSatCommBase::~InterSatCommBase()
{
  inter_sat_comm_->RemoveCommElement(this->name_tag_);
}
