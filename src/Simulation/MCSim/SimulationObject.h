#pragma once

#include <string>
#include <vector>
#include <map>
#include <functional>

#include <Library/math/Vector.hpp>
#include <Library/math/Quaternion.hpp>
#include "InitParameter.h"
#include "MCSimExecutor.h"

using libra::Vector;

class SimulationObject
{
public:
  // コンストラクタ
  explicit SimulationObject(std::string name);

  //// コピーコンストラクタ
  //SimulationObject(const SimulationObject& other);

  // デストラクタ
  virtual ~SimulationObject();

  //// 代入演算子
  //SimulationObject& operator=(const SimulationObject& other);

  //// アクセッサ
  //string GetName() const;

  //// 等価演算子
  //bool operator==(const SimulationObject& other) const;
  //bool operator!=(const SimulationObject& other) const;

  template<size_t NumElement>
  // Randomizeされた後の値を取得しdst_vecに格納
  void GetInitParameterVec(const MCSimExecutor& mc_sim, std::string ip_name, Vector<NumElement>& dst_vec) const;

  // Randomizeされた後の値を取得しdstに格納
  void GetInitParameterDouble(const MCSimExecutor& mc_sim, std::string ip_name, double& dst) const;

  // Randomizeされた後の値を取得しdst_quatに格納
  void GetInitParameterQuaternion(const MCSimExecutor& mc_sim, std::string ip_name, Quaternion& dst_quat) const;

  // Randomize結果を実際に使われる変数へ格納する処理を行う関数．継承先において定義されるべき純粋仮想関数．
  virtual void SetParameters(const MCSimExecutor& mc_sim) = 0;

  // 全てのSimulationObejectのSetParameter関数を実行する
  static void SetAllParameters(const MCSimExecutor& mc_sim);

private:
  // MCSim.iniにおいて区別するための名称．継承する際にはコンストラクタの引数として設定する．
  std::string name_;

  // list of objects with simulation parameters
  static std::map<std::string, SimulationObject*> so_list_;

};

template<size_t NumElement>
void SimulationObject::GetInitParameterVec(const MCSimExecutor& mc_sim, std::string ip_name, Vector<NumElement>& dst_vec) const
{
  mc_sim.GetInitParameterVec(name_, ip_name, dst_vec);
}
