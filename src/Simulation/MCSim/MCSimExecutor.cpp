#include "MCSimExecutor.h"

using std::string;

MCSimExecutor::MCSimExecutor(unsigned long long total_num_of_executions)
  :total_num_of_executions_(total_num_of_executions)
{
  num_of_executions_done_ = 0;
  enabled_ = total_num_of_executions_ > 1 ? true : false;
  log_history_ = !enabled_;
}

// 次のcaseを実行するかどうか
bool MCSimExecutor::WillExecuteNextCase()
{
  if (!enabled_)
  {
    return (num_of_executions_done_ < 1);
  }
  else
  {
    return (num_of_executions_done_ < total_num_of_executions_);
  }
}

void MCSimExecutor::AtTheBeginningOfEachCase()
{
  // Randomization結果のcsv出力など
  ;
}

void MCSimExecutor::AtTheEndOfEachCase()
{
  // Simulation結果のcsv出力など
  num_of_executions_done_++;
}

void MCSimExecutor::GetInitParameterDouble(string so_name, string ip_name, double& dst) const
{
  if (!enabled_) return;
  {
    string name = so_name + MCSimExecutor::separator_ + ip_name;
    if (ip_list_.find(name) == ip_list_.end())
    {
      // ip_listに登録されていない（MCSim.iniで定義されていない）
      return; // return without any update of dst
    }
    else
    {
      ip_list_.at(name)->GetDouble(dst);  // const mapなのでoperator[]は使えない
    }
  }
}

void MCSimExecutor::GetInitParameterQuaternion(string so_name, string ip_name, Quaternion& dst_quat) const
{
  if (!enabled_) return;
  {
    string name = so_name + MCSimExecutor::separator_ + ip_name;
    if (ip_list_.find(name) == ip_list_.end())
    {
      // ip_listに登録されていない（MCSim.iniで定義されていない）
      return; // return without any update of dst
    }
    else
    {
      ip_list_.at(name)->GetQuaternion(dst_quat);  // const mapなのでoperator[]は使えない
    }
  }
}

void MCSimExecutor::RandomizeAllParameters()
{
  for (auto ip : ip_list_)
  {
    ip.second->Randomize();
  }
}

void MCSimExecutor::SetSeed(unsigned long seed, bool is_deterministic)
{
  InitParameter::SetSeed(seed, is_deterministic);
}
