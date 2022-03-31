#pragma once
#include <Interface/SpacecraftInOut/Ports/GPIOPort.h>
#include <Interface/SpacecraftInOut/Ports/I2CPort.h>
#include <Interface/SpacecraftInOut/Ports/SCIPort.h>

#include <map>

#include "../Abstract/ComponentBase.h"

class OBC : public ComponentBase {
 public:
  OBC(ClockGenerator* clock_gen);
  OBC(int prescaler, ClockGenerator* clock_gen, PowerPort* power_port);
  OBC(int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, const double minimum_voltage, const double assumed_power_consumption);
  virtual ~OBC();

  // TODO:Rename the following functions to UartHogeHoge
  // UART Communication port functions
  virtual int ConnectComPort(int port_id, int tx_buf_size, int rx_buf_size);
  virtual int CloseComPort(int port_id);
  // UART OBC -> Components
  virtual int SendFromObc(int port_id, unsigned char* buffer, int offset, int count);
  virtual int ReceivedByCompo(int port_id, unsigned char* buffer, int offset, int count);
  // UART Components -> OBC
  virtual int SendFromCompo(int port_id, unsigned char* buffer, int offset, int count);
  virtual int ReceivedByObc(int port_id, unsigned char* buffer, int offset, int count);

  // I2C Communication port functions
  virtual int I2cConnectPort(int port_id, const unsigned char i2c_addr);
  virtual int I2cCloseComPort(int port_id);
  virtual int I2cComponentWriteRegister(int port_id, const unsigned char i2c_addr, const unsigned char reg_addr, const unsigned char* data,
                                        const unsigned char len);
  virtual int I2cComponentReadRegister(int port_id, const unsigned char i2c_addr, const unsigned char reg_addr, unsigned char* data,
                                       const unsigned char len);
  virtual int I2cComponentReadCommand(int port_id, const unsigned char i2c_addr, unsigned char* data, const unsigned char len);

  // GPIO port functions
  virtual int GpioConnectPort(int port_id);
  virtual int GpioComponentWrite(int port_id, const bool is_high);
  virtual bool GpioComponentRead(int port_id);  // return false when the port_id is not used

 protected:
  // function
  virtual void Initialize();
  virtual void MainRoutine(int count);

 private:
  // UART ports
  std::map<int, SCIPort*> com_ports_;
  // I2C ports
  std::map<int, I2CPort*> i2c_com_ports_;
  // GPIO ports
  std::map<int, GPIOPort*> gpio_ports_;
};
