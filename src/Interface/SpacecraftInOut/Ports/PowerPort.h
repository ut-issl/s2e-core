#pragma once

class PowerPort
{
public:
  PowerPort();
  PowerPort(int port_id, double current_Limit, double minimum_voltage, double assumed_power_consumption);
  ~PowerPort();

  bool Update(void);  //return is_on_

  // Getters
  inline const double GetVoltage(void)const {return voltage_;};
  inline const double GetCurrentConsumption() const {return current_consumption_;};
  inline const double GetAssumedPowerConsumption() const {return assumed_power_consumption_;};
  inline const bool GetIsOn() const {return is_on_;};

  // Setters
  bool SetVoltage(const double voltage);  //return is_on_
  inline void SetAssumedPowerConsumption(const double power){assumed_power_consumption_ = power;};
  
private:
  const int kPortId;
  const double kCurrentLimit; // [A] for Over Current Protection
  const double kMinimumVoltage; // [V] Minimum voltage for power on
  
  // Assumed power consumption when the switch is turned on
  double assumed_power_consumption_; // [W]
  
  double voltage_; // [V]
  double current_consumption_; // calculated by I = P/V[A]
  bool is_on_;

  void Initialize(void);
};

