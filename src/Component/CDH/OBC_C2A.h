#pragma once
#include <Interface/SpacecraftInOut/Ports/GPIOPort.h>

#include "OBC.h"

class OBC_C2A : public OBC {
 public:
  OBC_C2A(ClockGenerator* clock_gen);
  OBC_C2A(ClockGenerator* clock_gen, int timing_regulator);
  OBC_C2A(int prescaler, ClockGenerator* clock_gen, int timing_regulator,
          PowerPort* power_port);
  ~OBC_C2A();

  // Communication port functions
  int ConnectComPort(int port_id, int tx_buf_size, int rx_buf_size) override;
  int CloseComPort(int port_id) override;
  // OBC -> Components
  int SendFromObc(int port_id, unsigned char* buffer, int offset,
                  int count) override;
  int ReceivedByCompo(int port_id, unsigned char* buffer, int offset,
                      int count) override;
  // Components -> OBC
  int SendFromCompo(int port_id, unsigned char* buffer, int offset,
                    int count) override;
  int ReceivedByObc(int port_id, unsigned char* buffer, int offset,
                    int count) override;
  // Static function for C2A
  static int SendFromObc_C2A(int port_id, unsigned char* buffer, int offset,
                             int count);
  static int ReceivedByObc_C2A(int port_id, unsigned char* buffer, int offset,
                               int count);

  // I2C
  int I2cConnectPort(int port_id, const unsigned char i2c_addr) override;
  int I2cCloseComPort(int port_id) override;
  // For Component
  int I2cComponentWriteRegister(int port_id, const unsigned char i2c_addr,
                                const unsigned char reg_addr,
                                const unsigned char* data,
                                const unsigned char len) override;
  int I2cComponentReadRegister(int port_id, const unsigned char i2c_addr,
                               const unsigned char reg_addr,
                               unsigned char* data,
                               const unsigned char len) override;
  int I2cComponentReadCommand(int port_id, const unsigned char i2c_addr,
                              unsigned char* data,
                              const unsigned char len) override;

  // Static function for C2A
  static int I2cWriteCommand(int port_id, const unsigned char i2c_addr,
                             const unsigned char* data,
                             const unsigned char len);
  static int I2cWriteRegister(int port_id, const unsigned char i2c_addr,
                              const unsigned char* data,
                              const unsigned char len);
  static int I2cReadRegister(int port_id, const unsigned char i2c_addr,
                             unsigned char* data, const unsigned char len);

  // GPIO
  int GpioConnectPort(int port_id) override;
  int GpioComponentWrite(int port_id, const bool is_high) override;
  bool GpioComponentRead(
      int port_id) override;  // return false when the port_id is not used
  // Static function for C2A
  static int GpioWrite_C2A(int port_id, const bool is_high);
  static bool GpioRead_C2A(
      int port_id);  // return false when the port_id is not used

 private:
  bool is_initialized = false;
  const int timing_regulator_;
  void MainRoutine(int count);
  void Initialize();
  // ports
  static std::map<int, SCIPort*> com_ports_c2a_;
  static std::map<int, I2CPort*> i2c_com_ports_c2a_;
  static std::map<int, GPIOPort*> gpio_ports_c2a_;
};

// C2A communication functions
// If the character encoding of C2A is UTF-8, these functions are not necessary,
// and users can directory use SendFromObc_C2A and ReceivedByObc_C2A UART
int OBC_C2A_SendFromObc(int port_id, unsigned char* buffer, int offset,
                        int count);
int OBC_C2A_ReceivedByObc(int port_id, unsigned char* buffer, int offset,
                          int count);

// I2C
int OBC_C2A_I2cWriteCommand(int port_id, const unsigned char i2c_addr,
                            const unsigned char* data, const unsigned char len);
int OBC_C2A_I2cWriteRegister(int port_id, const unsigned char i2c_addr,
                             const unsigned char* data,
                             const unsigned char len);
int OBC_C2A_I2cReadRegister(int port_id, const unsigned char i2c_addr,
                            unsigned char* data, const unsigned char len);

// GPIO
int OBC_C2A_GpioWrite(int port_id, const bool is_high);
bool OBC_C2A_GpioRead(
    int port_id);  // return false when the port_id is not used
