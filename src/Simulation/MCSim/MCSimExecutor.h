#pragma once

#include <string>
#include <map>
#include <Library/math/Vector.hpp>
//#include "SimulationObject.h"
#include "InitParameter.h"

using libra::Vector;

class MCSimExecutor
{
private:
  unsigned long long total_num_of_executions_;  // シミュレーションケース数
  unsigned long long num_of_executions_done_;  // 実行済みケース数
  bool enabled_;  // Monte-Carlo Simulationを行うかどうかのflag
  bool log_history_;  // 1 caseごとのログを記録するかどうかのflag

  // List of InitParameters read from MCSim.ini
  std::map<std::string, InitParameter*> ip_list_;

public:
  // MCSim.iniにおいてSimulationObjectとInitParameter名を区別するための文字
  static const char separator_ = '.';

  // コンストラクタ
  MCSimExecutor(unsigned long long total_num_of_executions);

  // enable_のsetter
  inline void Enable(bool enabled);

  // enable_のgetter
  inline bool IsEnabled() const;

  // total_num_of_executions_のsetter
  inline void SetTotalNumOfExecutions(unsigned long long num_of_executions);

  // total_num_of_executions_のgetter
  inline unsigned long long GetTotalNumOfExecutions() const;

  inline unsigned long long GetNumOfExecutionsDone() const;

  // log_historyのsetter
  inline void LogHistory(bool set);

  // log_history_のgetter
  inline bool LogHistory() const;

  // 次のcaseを実行するかどうかの判定
  bool WillExecuteNextCase();

  // 毎caseのrandomization後，Simulation実行前に行う処理（Randomiza結果のログ出力などを想定）
  void AtTheBeginningOfEachCase();

  // 毎caseのSimulation実行後に行う処理（Simulation結果（成功or失敗など）のログ出力などを想定）
  void AtTheEndOfEachCase();

  // Randomization seedの設定．is_deterministic = falaseの場合は引数seedは用いられず時刻情報が用いられる．
  static void SetSeed(unsigned long seed = 0, bool is_deterministic = false);

  template<size_t NumElement1, size_t NumElement2>
  // InitParameterの追加
  void AddInitParameter(std::string so_name, std::string ip_name, const Vector<NumElement1>& mean_or_min, const Vector<NumElement2>& sigma_or_max, InitParameter::RandomizationType rnd_type);

  // 全てのInitParameterをRandomize
  void RandomizeAllParameters();

  template<size_t NumElement>
  // Randomizeされた後の値を取得しdst_vecに格納
  void GetInitParameterVec(std::string so_name, std::string ip_name, Vector<NumElement>& dst_vec) const;

  // Randomizeされた後の値を取得しdstに格納
  void GetInitParameterDouble(std::string so_name, std::string ip_name, double& dst) const;

  // Randomizeされた後の値を取得しdst_quatに格納
  void GetInitParameterQuaternion(std::string so_name, std::string ip_name, Quaternion& dst_quat) const;
};

void MCSimExecutor::Enable(bool enabled)
{
  enabled_ = enabled;
}

bool MCSimExecutor::IsEnabled() const
{
  return enabled_;
}

void MCSimExecutor::SetTotalNumOfExecutions(unsigned long long num_of_executions)
{
  total_num_of_executions_ = num_of_executions;
}

unsigned long long MCSimExecutor::GetTotalNumOfExecutions() const
{
  return total_num_of_executions_;
}

inline unsigned long long MCSimExecutor::GetNumOfExecutionsDone() const
{
  return num_of_executions_done_;
}

// 1回1回のSimulationログを記録するか
bool MCSimExecutor::LogHistory() const
{
  // MCSimでないか，LogHistory=ENABLEDの場合にはログをとる
  return (!enabled_ || log_history_);
}

void MCSimExecutor::LogHistory(bool set)
{
  log_history_ = set;
}

template<size_t NumElement>
void MCSimExecutor::GetInitParameterVec(std::string so_name, std::string ip_name, Vector<NumElement>& dst_vec) const
{
  if (!enabled_) return;
  std::string name = so_name + MCSimExecutor::separator_ + ip_name;
  if (ip_list_.find(name) == ip_list_.end())
  {
    // ip_listに登録されていない（MCSim.iniで定義されていない）
    return; // return without update the dst_vec
  }
  else
  {
    ip_list_.at(name)->GetVec(dst_vec); // const mapなのでoperator[]は使えない
  }
}

template<size_t NumElement1, size_t NumElement2>
void MCSimExecutor::AddInitParameter(std::string so_name, std::string ip_name, const Vector<NumElement1>& mean_or_min, const Vector<NumElement2>& sigma_or_max, InitParameter::RandomizationType rnd_type)
{
  std::string name = so_name + MCSimExecutor::separator_ + ip_name;
  if (ip_list_.find(name) == ip_list_.end())
  {
    // ip_listに登録されていない場合は登録
    auto newparam = new InitParameter();
    newparam->SetRandomConfig(mean_or_min, sigma_or_max, rnd_type);
    ip_list_[name] = newparam;
  }
  else
  {
    // 既に登録されている場合はエラー
    throw "More than one definition of one InitParameter.";
  }
}
