/*
 * @file on_board_computer_with_c2a.cpp
 * @brief Class to emulate on board computer with C2A flight software
 */

#include "on_board_computer_with_c2a.hpp"
#include <iostream>
#include <regex>
#include <string>

#ifdef USE_C2A
#include "library/initialize/c2a_command_database.hpp"
#include "src_core/c2a_core_main.h"

#if C2A_CORE_VER_MAJOR == 4
// c2a-core v4
#include "src_core/system/task_manager/task_dispatcher.h"
#include "src_core/system/time_manager/time_manager.h"
#include "src_core/system/watchdog_timer/watchdog_timer.h"
#elif C2A_CORE_VER_MAJOR <= 3
// c2a-core <= v3
#include "src_core/Library/endian.h"
#include "src_core/System/TaskManager/task_dispatcher.h"
#include "src_core/System/TimeManager/time_manager.h"
#include "src_core/System/WatchdogTimer/watchdog_timer.h"
#include "src_core/TlmCmd/command_analyze.h"
#include "src_core/TlmCmd/common_cmd_packet_util.h"
#include "src_user/TlmCmd/command_definitions.h"
#else
#error "c2a-core version is not supported"
#endif  // c2a-core version header

#endif  // USE_C2A

std::map<int, UartPort*> ObcWithC2a::com_ports_c2a_;
std::map<int, I2cPort*> ObcWithC2a::i2c_com_ports_c2a_;
std::map<int, GpioPort*> ObcWithC2a::gpio_ports_c2a_;

ObcWithC2a::ObcWithC2a(ClockGenerator* clock_generator) : OnBoardComputer(clock_generator), timing_regulator_(1) {
  // Initialize();
}

ObcWithC2a::ObcWithC2a(ClockGenerator* clock_generator, int timing_regulator)
    : OnBoardComputer(clock_generator), timing_regulator_(timing_regulator) {
  // Initialize();
}

ObcWithC2a::ObcWithC2a(int prescaler, ClockGenerator* clock_generator, int timing_regulator, PowerPort* power_port)
    : OnBoardComputer(prescaler, clock_generator, power_port), timing_regulator_(timing_regulator) {
  // Initialize();
}

ObcWithC2a::~ObcWithC2a() { delete command_database_; }

void ObcWithC2a::Initialize() {
#ifdef USE_C2A
  TMGR_init();  // Time Manager
                // Initialize at the beginning in order to measure the execution
                // time of C2A core initialization.
  C2A_core_init();
  WDT_init();  // Watchdog timer. In SILS, it does not have meaning.

  TMGR_clear();  // This called in C2A_core_init, but should be called again
                 // just before executing the C2A main loop.
  command_database_ = new C2aCommandDatabase("../../data/initialize_files/components/ISSL6U_AOBC_CMD_DB_CMD_DB.csv");
#endif
}

void ObcWithC2a::MainRoutine(const int time_count) {
  UNUSED(time_count);

#ifdef USE_C2A
  if (is_initialized == false) {
    is_initialized = true;
    Initialize();
  }
  for (int i = 0; i < timing_regulator_; i++) {
    TMGR_count_up_master_clock();  // The update time oc C2A clock should be 1msec
    TDSP_execute_pl_as_task_list();
  }
  RegisterCommand();
#else
  UNUSED(is_initialized);
  UNUSED(timing_regulator_);
#endif
}

void ObcWithC2a::RegisterCommand() {
  AnalyzeCommandLine(".AOBC_RT.Cmd_CODE_APP_AOCS_MANAGER_SET_MASS 22.6 # MPU TLM ERROR");
}

void ObcWithC2a::AnalyzeCommandLine(const std::string input_line) {
  // comment削除
  std::string cmd_line = input_line;
  size_t comment_position = cmd_line.find('#');
  if (comment_position != std::string::npos) {
    cmd_line = cmd_line.substr(0, comment_position);
  }

  // ポーズポイント削除
  if (!cmd_line.empty() && cmd_line[0] == '.') {
    cmd_line = cmd_line.substr(1);
  }

  // スペース分割
  std::istringstream tokenStream(cmd_line);
  std::string token;
  std::vector<std::string> tokens;
  while (tokenStream >> token) {
    tokens.push_back(token);
  }

  // コマンド認識
  if (tokens[0].find("AOBC_RT") == 0) {  // FIXME AOBC以外も対応する
    // C2Aコマンドとして処理
    // Command Code 取得
    std::string cmd_enum_name = tokens[0].substr(8);
    cmd_enum_name = "Cmd" + cmd_enum_name.substr(8);
    C2aCommandInformation cmd_info = command_database_->GetCommandInformation(cmd_enum_name);
    if (cmd_info.GetCommandName() == "Error") {
      // TODO エラー処理
    }
    CMD_CODE cmd_id = (CMD_CODE)cmd_info.GetCommandId();

    // 引数取得
    std::vector<std::string> arguments;
    for (size_t i = 1; i < tokens.size(); i++) {
      arguments.push_back(tokens[i]);
    }
    // 引数個数確認
    if (arguments.size() != cmd_info.GetNumberOfArgument()) {
      // TODO エラー処理
    }

    // 引数処理
    uint8_t param[CSP_MAX_LEN];
    uint16_t param_len = 0;
    for (size_t arg_num = 0; arg_num < arguments.size(); arg_num++) {
      size_t len = 0;
      DecodeC2aCommandArgument(cmd_info.GetArgumentType(arg_num), arguments[arg_num], param + param_len, len);
      param_len += (uint16_t)len;
    }
    // コマンド送信
    CCP_register_rtc(cmd_id, param, param_len);

  } else if (tokens[0].find("wait") == 0) {
    // wait として処理
  } else {
    // 処理しない
  }

  // debug出力
  for (const std::string& t : tokens) {
    std::cout << t << std::endl;
  }
}

// Override functions
int ObcWithC2a::ConnectComPort(int port_id, int tx_buffer_size, int rx_buffer_size) {
  if (com_ports_c2a_[port_id] != nullptr) {
    // Port already used
    return -1;
  }
  com_ports_c2a_[port_id] = new UartPort(tx_buffer_size, rx_buffer_size);
  return 0;
}

// Close port and free resources
int ObcWithC2a::CloseComPort(int port_id) {
  // Port not used
  if (com_ports_c2a_[port_id] == nullptr) return -1;

  UartPort* port = com_ports_c2a_.at(port_id);
  delete port;
  com_ports_c2a_.erase(port_id);
  return 0;
}

int ObcWithC2a::SendFromObc(int port_id, unsigned char* buffer, int offset, int length) {
  return ObcWithC2a::SendFromObc_C2A(port_id, buffer, offset, length);
}

int ObcWithC2a::ReceivedByCompo(int port_id, unsigned char* buffer, int offset, int length) {
  UartPort* port = com_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->ReadTx(buffer, offset, length);
}

int ObcWithC2a::SendFromCompo(int port_id, unsigned char* buffer, int offset, int length) {
  UartPort* port = com_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->WriteRx(buffer, offset, length);
}

int ObcWithC2a::ReceivedByObc(int port_id, unsigned char* buffer, int offset, int length) {
  return ObcWithC2a::ReceivedByObc_C2A(port_id, buffer, offset, length);
}

// Static functions
int ObcWithC2a::SendFromObc_C2A(int port_id, unsigned char* buffer, int offset, int length) {
  UartPort* port = com_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->WriteTx(buffer, offset, length);
}
int ObcWithC2a::ReceivedByObc_C2A(int port_id, unsigned char* buffer, int offset, int length) {
  UartPort* port = com_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->ReadRx(buffer, offset, length);
}

// If the character encoding of C2A is UTF-8, these functions are not necessary,
// and users can directory use SendFromObc_C2A and ReceivedByObc_C2A
int OBC_C2A_SendFromObc(int port_id, unsigned char* buffer, int offset, int length) {
  return ObcWithC2a::SendFromObc_C2A(port_id, buffer, offset, length);
}
int OBC_C2A_ReceivedByObc(int port_id, unsigned char* buffer, int offset, int length) {
  return ObcWithC2a::ReceivedByObc_C2A(port_id, buffer, offset, length);
}

int ObcWithC2a::I2cConnectPort(int port_id, const unsigned char i2c_address) {
  if (i2c_com_ports_c2a_[port_id] != nullptr) {
    // Port already used
  } else {
    i2c_com_ports_c2a_[port_id] = new I2cPort();
  }
  i2c_com_ports_c2a_[port_id]->RegisterDevice(i2c_address);

  return 0;
}

int ObcWithC2a::I2cCloseComPort(int port_id) {
  // Port not used
  if (i2c_com_ports_c2a_[port_id] == nullptr) return -1;

  I2cPort* port = i2c_com_ports_c2a_.at(port_id);
  delete port;
  i2c_com_ports_c2a_.erase(port_id);
  return 0;
}

int ObcWithC2a::I2cWriteCommand(int port_id, const unsigned char i2c_address, const unsigned char* data, const unsigned char length) {
  I2cPort* i2c_port = i2c_com_ports_c2a_[port_id];
  i2c_port->WriteCommand(i2c_address, data, length);
  return 0;
}

int ObcWithC2a::I2cWriteRegister(int port_id, const unsigned char i2c_address, const unsigned char* data, const unsigned char length) {
  I2cPort* i2c_port = i2c_com_ports_c2a_[port_id];

  if (length == 1) {
    i2c_port->WriteRegister(i2c_address, data[0]);
  } else {
    for (unsigned char i = 0; i < length - 1; i++) {
      i2c_port->WriteRegister(i2c_address, data[0] + i, data[i + 1]);
    }
  }
  return 0;
}

int ObcWithC2a::I2cReadRegister(int port_id, const unsigned char i2c_address, unsigned char* data, const unsigned char length) {
  I2cPort* i2c_port = i2c_com_ports_c2a_[port_id];
  for (int i = 0; i < length; i++) {
    data[i] = i2c_port->ReadRegister(i2c_address);
  }
  return 0;
}

int ObcWithC2a::I2cComponentWriteRegister(int port_id, const unsigned char i2c_address, const unsigned char register_address,
                                          const unsigned char* data, const unsigned char length) {
  I2cPort* i2c_port = i2c_com_ports_c2a_[port_id];
  for (unsigned char i = 0; i < length; i++) {
    i2c_port->WriteRegister(i2c_address, register_address + i, data[i]);
  }
  return 0;
}
int ObcWithC2a::I2cComponentReadRegister(int port_id, const unsigned char i2c_address, const unsigned char register_address, unsigned char* data,
                                         const unsigned char length) {
  I2cPort* i2c_port = i2c_com_ports_c2a_[port_id];
  for (unsigned char i = 0; i < length; i++) {
    data[i] = i2c_port->ReadRegister(i2c_address, register_address + i);
  }
  return 0;
}
int ObcWithC2a::I2cComponentReadCommand(int port_id, const unsigned char i2c_address, unsigned char* data, const unsigned char length) {
  I2cPort* i2c_port = i2c_com_ports_c2a_[port_id];
  i2c_port->ReadCommand(i2c_address, data, length);
  return 0;
}

int OBC_C2A_I2cWriteCommand(int port_id, const unsigned char i2c_address, const unsigned char* data, const unsigned char length) {
  return ObcWithC2a::I2cWriteCommand(port_id, i2c_address, data, length);
}
int OBC_C2A_I2cWriteRegister(int port_id, const unsigned char i2c_address, const unsigned char* data, const unsigned char length) {
  return ObcWithC2a::I2cWriteRegister(port_id, i2c_address, data, length);
}
int OBC_C2A_I2cReadRegister(int port_id, const unsigned char i2c_address, unsigned char* data, const unsigned char length) {
  return ObcWithC2a::I2cReadRegister(port_id, i2c_address, data, length);
}

int ObcWithC2a::GpioConnectPort(int port_id) {
  if (gpio_ports_c2a_[port_id] != nullptr) {
    // Port already used
    return -1;
  }
  gpio_ports_c2a_[port_id] = new GpioPort(port_id);
  return 0;
}

int ObcWithC2a::GpioComponentWrite(int port_id, const bool is_high) {
  GpioPort* port = gpio_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->DigitalWrite(is_high);
}

bool ObcWithC2a::GpioComponentRead(int port_id) {
  GpioPort* port = gpio_ports_c2a_[port_id];
  if (port == nullptr) return false;
  return port->DigitalRead();
}

int ObcWithC2a::GpioWrite_C2A(int port_id, const bool is_high) {
  GpioPort* port = gpio_ports_c2a_[port_id];
  if (port == nullptr) return -1;
  return port->DigitalWrite(is_high);
}

bool ObcWithC2a::GpioRead_C2A(int port_id) {
  GpioPort* port = gpio_ports_c2a_[port_id];
  if (port == nullptr) return false;
  return port->DigitalRead();
}

int OBC_C2A_GpioWrite(int port_id, const bool is_high) { return ObcWithC2a::GpioWrite_C2A(port_id, is_high); }

bool OBC_C2A_GpioRead(int port_id) { return ObcWithC2a::GpioRead_C2A(port_id); }