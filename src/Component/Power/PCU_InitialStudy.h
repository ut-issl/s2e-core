#pragma once

#include <Interface/LogOutput/ILoggable.h>

#include <vector>

#include "../Abstract/ComponentBase.h"
#include "BAT.h"
#include "SAP.h"

class PCU_InitialStudy : public ComponentBase, public ILoggable {
 public:
  PCU_InitialStudy(const int prescaler, ClockGenerator* clock_gen, const std::vector<SAP*> saps, BAT* bat, double compo_step_time);
  ~PCU_InitialStudy();

  /*LOG出力用関数*/
  std::string GetLogHeader() const override;
  std::string GetLogValue() const override;

 private:
  const std::vector<SAP*> saps_;
  BAT* const bat_;
  const double cc_charge_current_;  //[A]
  const double cv_charge_voltage_;  //[V]
  double bus_voltage_;              //[V]
  double power_consumption_;        //[W]
  double compo_step_time_;          //[sec]

  void MainRoutine(int time_count) override;
  double CalcPowerConsumption(double time_query) const;
  void UpdateChargeCurrentAndBusVoltage();
};
