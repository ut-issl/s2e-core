/**
 * @file InitHardwareMessage.cpp
 * @brief Initialize function for HardwareMessage
 */

#include "../hils/HardwareMessage.h"
#include "Initialize.h"

/**
 * @fn Init_HardwareMessage
 * @brief Initialize function for HardwareMessage
 * @param [in] file_name: File nama of the initialize file
 */
HardwareMessage* Init_HardwareMessage(string file_name) {
  IniAccess ini_file(file_name);

  char* section = "ObcDebugCom";
  bool receive_msg_from_obc = ini_file.ReadEnable(section, "ReceiveMsgFromObc");
  unsigned int obc_com_period = ini_file.ReadInt(section, "ObcComPeriod");
  unsigned int baudrate = ini_file.ReadInt(section, "BaudRate");
  unsigned short obc_com_port_num = ini_file.ReadInt(section, "ComPortNum");

  HardwareMessage* hw_msg = new HardwareMessage(obc_com_port_num, receive_msg_from_obc, baudrate, obc_com_period);
  hw_msg->IsLogEnabled = receive_msg_from_obc;
  return hw_msg;
}