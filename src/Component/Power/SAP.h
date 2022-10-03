#pragma once

#include <Environment/Local/LocalCelestialInformation.h>
#include <Environment/Local/SRPEnvironment.h>
#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/Vector.hpp>

#include "../Abstract/ComponentBase.h"

class SAP : public ComponentBase, public ILoggable {
 public:
  SAP(const int prescaler, ClockGenerator* clock_gen, int id, int number_of_series, int number_of_parallel, double cell_area, libra::Vector<3> normal_vector,
      double cell_efficiency, double transmission_efficiency, const SRPEnvironment* srp, const LocalCelestialInformation* local_celes_info, double compo_step_time);
  SAP(const int prescaler, ClockGenerator* clock_gen, int id, int number_of_series, int number_of_parallel, double cell_area, libra::Vector<3> normal_vector,
      double cell_efficiency, double transmission_efficiency, const SRPEnvironment* srp, double compo_step_time);
  SAP(const SAP& obj);
  ~SAP();
  double GetPowerGeneration() const;
  void SetVoltage(const double voltage);

  /*LOG出力用関数*/
  std::string GetLogHeader() const override;
  std::string GetLogValue() const override;

 private:
  const int id_;  //ここは文字列にした方が分かりやすい？
  const int number_of_series_;
  const int number_of_parallel_;
  const double cell_area_;  //[m^2]
  const libra::Vector<3> normal_vector_;
  const double cell_efficiency_;
  const double transmission_efficiency_;  //各種損失を考慮したPCUへの伝達効率
  const SRPEnvironment* const srp_;
  const LocalCelestialInformation* local_celes_info_;
  double voltage_;           //[V]
  double power_generation_;  //[W]
  /* 他にIV曲線を決めるために必要なパラメータなど
  コンポ依存のパラメータをconstで持つ
  太陽光強度などによって変化するパラメータは非constで持ち，MainRoutineで更新 */
  static const double solar_constant_;
  static const double light_speed_;
  double compo_step_time_;   //[sec]

  void MainRoutine(int time_count) override;
};
