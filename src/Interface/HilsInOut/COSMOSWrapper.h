#pragma once

#include <string>

#include "COSMOS_TCP_IF.h"

class COSMOSWrapper : private COSMOS_TCP_IF  // SendString関数を使う目的で継承
{
 public:
  COSMOSWrapper(bool enable);
  COSMOSWrapper(unsigned short port_num, bool enable, char* server_ip);
  COSMOSWrapper();  // when not in use
  ~COSMOSWrapper();
  void Cmd(std::string cmd_str);
  void Cmd_RDP_SAT_CMD_EXTERNAL_TORQUE(uint8_t torque_frame, uint32_t duration_ms, float ext_torque_x, float ext_torque_y, float ext_torque_z);
  void Initialize();
  void Finalize();
  bool Enable() const;

 private:
  void CosmosEval(std::string expr);
  const bool enable_;
};
