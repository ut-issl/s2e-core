#pragma once

#include <Interface/LogOutput/ILoggable.h>

#include <vector>

#include "../Abstract/ComponentBase.h"

class BAT : public ComponentBase, public ILoggable {
 public:
  BAT(const int prescaler, ClockGenerator* clock_gen, int number_of_series, int number_of_parallel, double cell_capacity,
      const std::vector<double> cell_discharge_curve_coeffs, double initial_dod, double cc_charge_c_rate, double cv_charge_voltage,
      double bat_resistance, double compo_step_time);
  BAT(const BAT& obj);
  ~BAT();
  void SetChargeCurrent(const double current);
  double GetBatVoltage() const;
  double GetBatResistance() const;
  double GetCCChargeCurrent() const;  //今後実装方法は変える？
  double GetCVChargeVoltage() const;  //今後実装方法は変える？

  /*LOG出力用関数*/
  std::string GetLogHeader() const override;
  std::string GetLogValue() const override;

 private:
  const int number_of_series_;
  const int number_of_parallel_;
  const double cell_capacity_;  //[Ah]
  const std::vector<double> cell_discharge_curve_coeffs_;
  const double cc_charge_current_;  //[C]
  const double cv_charge_voltage_;  //[V]
  double bat_voltage_;              //[V]
  double dod_;                      //[%]
  double charge_current_;           //[A]
  double bat_resistance_;           //[Ohm]
  double compo_step_time_;          //[sec]

  void MainRoutine(int count) override;
  void UpdateBatVoltage();
};
