#pragma once

#include "../LogOutput/ILoggable.h"
#include <Library/utils/endian.h>
#include "ComPortInterface.h"

typedef unsigned short int crc_t;

// 以下のenumはC2AのValveControl.hと整合性をとる
typedef enum
{
  VALVE_THRUSTER_DVT1,       // 0
  VALVE_THRUSTER_DVT2,       // 1
  VALVE_THRUSTER_RCT1,       // 2
  VALVE_THRUSTER_RCT2,       // 3
  VALVE_THRUSTER_RCT3,       // 4
  VALVE_THRUSTER_RCT4,       // 5
  VALVE_THRUSTER_MAX
} VALVE_THRUSTER_ENUM;

class HardwareMessage : public ILoggable
{
public:
  HardwareMessage(int port_id, bool enable, unsigned int baudrate, unsigned int obc_com_period = 1);
  HardwareMessage(); // when not in use
  ~HardwareMessage();
  int ReceiveAndInterpretMsg();
  //HWMSG* hw_msg; // ここに書くとHWMSGが未定義になる
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:
  ComPortInterface* hils_com_port_;
  const bool enable_;
  unsigned int obc_com_period_;
  const int kBaudRate;
  static const unsigned int kTxMessageSize = 1; // 基本送らない
  static const unsigned int kRxMessageSize = 256;
  static const size_t kHeaderSize = 2;
  static const size_t kFooterSize = 2;
  const uint8_t kHeader[kHeaderSize] = { 0xBE, 0xEF };
  const uint8_t kFooter[kFooterSize] = { 0xCA, 0xFE };
  uint8_t txbuff[kTxMessageSize];
  uint8_t rxbuff[kRxMessageSize * 2]; // HACK: ヘッダーが見つからない場合のため、とりあえず2倍用意しておく

  static void ZeroPad(void* ptr, size_t size);

  struct HWMSG {
    uint8_t header[kHeaderSize];
    uint32_t obc_ti;
    struct SADA {
      uint8_t limit_switch_status;
      int32_t position_steps;
      int16_t position_degrees;
      uint16_t step_period;
      uint32_t steps_remaining;
      uint64_t steps_performed;
      uint64_t steps_since_home;
      uint8_t spare[3];
    } sada;
    struct VALVE_CTRL {
      uint8_t TV_idx;
      uint32_t cumulative_rv_open_counter;
      double thruster_thrust[VALVE_THRUSTER_MAX];
      uint8_t spare[11];
    } valve_ctrl;
    struct THERMAL_CTRL {
      uint8_t spare[64];
    } thermal_ctrl;
    uint8_t spare[86];
    crc_t crc;
    uint8_t footer[kFooterSize];
  } hw_msg_;
public:
  const HWMSG* hw_msg = &hw_msg_; // これで外部から書き換え禁止のアクセスできるか？
  const ComPortInterface* hils_com_port = hils_com_port_;
  bool Enable() const;
  int Initialize();
};
