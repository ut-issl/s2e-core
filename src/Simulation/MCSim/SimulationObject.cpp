#include "SimulationObject.h"

std::map<std::string, SimulationObject*> SimulationObject::so_list_;

SimulationObject::SimulationObject(std::string name)
  :name_(name)
{
  // so_listに登録されているかを調べる
  std::map<std::string, SimulationObject*>::iterator itr = SimulationObject::so_list_.find(name);

  if (itr == SimulationObject::so_list_.end())
  {
    // so_listに未登録の場合，自分を登録する
    SimulationObject::so_list_[name] = this;
  }
  else
  {
    // so_listに登録済みの場合．前回のcase終了時にdeleteされていないということなのでエラーを吐く
    // または（考えにくいが）同名のSimulationObjectが2つ以上定義されている可能性もある．
    throw "SimulationObject already defined. You may have forgotten to delete me in the previous simulation case, or defined two SimulationObjects with the same name.";
  }
}

SimulationObject::~SimulationObject()
{
  // so_listから自分を削除
  SimulationObject::so_list_.erase(name_);
}

void SimulationObject::SetAllParameters(const MCSimExecutor& mc_sim)
{
  for (auto so : SimulationObject::so_list_)
  {
    so.second->SetParameters(mc_sim);
  }
}

void SimulationObject::GetInitParameterDouble(const MCSimExecutor& mc_sim, std::string ip_name, double& dst) const
{
  mc_sim.GetInitParameterDouble(name_, ip_name, dst);
}

void SimulationObject::GetInitParameterQuaternion(const MCSimExecutor& mc_sim, std::string ip_name, Quaternion& dst_quat) const
{
  mc_sim.GetInitParameterQuaternion(name_, ip_name, dst_quat);
}
