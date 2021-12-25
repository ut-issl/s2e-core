#pragma once

#include <vector>
#include "../Abstract/ComponentBase.h"
#include "../../Interface/LogOutput/ILoggable.h"

class BAT : public ComponentBase, public ILoggable
{
public:
  BAT(int number_of_series,
    int number_of_parallel,
    double cell_capacity,
    const std::vector<double> cell_discharge_curve_coeffs,
    double initial_dod,
    double cc_charge_c_rate,
    double cv_charge_voltage,
    double bat_resistance);
    BAT(const BAT &obj);
  ~BAT();
  void SetChargeCurrent(const double current);
  double GetBatVoltage() const;
  double GetBatResistance() const;
  double GetCCChargeCurrent() const; //今後実装方法は変える？
  double GetCVChargeVoltage() const; //今後実装方法は変える？

  /*LOG出力用関数*/
  string GetLogHeader() const override;
  string GetLogValue() const override;

private:
  const int number_of_series_;
  const int number_of_parallel_;
  const double cell_capacity_; //[Ah]
  const std::vector<double> cell_discharge_curve_coeffs_;
  const double cc_charge_current_; //[C]
  const double cv_charge_voltage_; //[V]
  double bat_voltage_; //[V]
  double dod_; //[%]
  double charge_current_; //[A]
  double bat_resistance_; //[Ohm]

  void MainRoutine(int time_count) override;
  void UpdateBatVoltage();
};