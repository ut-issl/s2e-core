#pragma once

class PowerPort
{
public:
  PowerPort();
  PowerPort(int port_id, double current_Limit);
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
  inline void SetMinimumVoltage(const double minimum_voltage){minimum_voltage_ = minimum_voltage;}; 
  inline void SetCurrentLimit(const double current_limit){current_limit_ = current_limit;}; 
private:
  // PCU setting parameters
  const int kPortId;
  double current_limit_; // [A] for Over Current Protection
  
  // Components setting parameters
  double minimum_voltage_; // [V] Minimum voltage for power on
  double assumed_power_consumption_; // [W] Assumed power consumption when the switch is turned on
  
  double voltage_; // [V]
  double current_consumption_; // calculated by I = P/V[A]
  bool is_on_;

  void Initialize(void);
};

