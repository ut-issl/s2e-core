﻿#pragma once

#include "../../Interface/LogOutput/ILoggable.h"
#include "../Abstract/ComponentBase.h"
#include "BAT.h"
#include "SAP.h"
#include <vector>

class PCU_InitialStudy : public ComponentBase, public ILoggable {
public:
  PCU_InitialStudy(ClockGenerator *clock_gen, const std::vector<SAP *> saps,
                   BAT *bat);
  ~PCU_InitialStudy();

  /*LOG出力用関数*/
  std::string GetLogHeader() const override;
  std::string GetLogValue() const override;

private:
  const std::vector<SAP *> saps_;
  BAT *const bat_;
  const double cc_charge_current_; //[A]
  const double cv_charge_voltage_; //[V]
  double bus_voltage_;             //[V]
  double power_consumption_;       //[W]

  void MainRoutine(int time_count) override;
  double CalcPowerConsumption(int time_in_sec) const;
  void UpdateChargeCurrentAndBusVoltage();
};
