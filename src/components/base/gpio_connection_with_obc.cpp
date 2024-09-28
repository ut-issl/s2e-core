/**
 * @file gpio_connection_with_obc.cpp
 * @brief Base class for GPIO communication with OBC flight software
 *        TODO: consider relation with IGPIOCompo
 */

#include "gpio_connection_with_obc.hpp"

namespace s2e::components {

GpioConnectionWithObc::GpioConnectionWithObc(const std::vector<int> port_id, OnBoardComputer* obc) : port_id_(port_id), obc_(obc) {
  for (size_t i = 0; i < port_id_.size(); i++) {
    obc_->GpioConnectPort(port_id_[i]);
  }
}

GpioConnectionWithObc::~GpioConnectionWithObc() {}

bool GpioConnectionWithObc::Read(const int index) { return obc_->GpioComponentRead(port_id_[index]); }

void GpioConnectionWithObc::Write(const int index, const bool is_high) { obc_->GpioComponentWrite(port_id_[index], is_high); }

}  // namespace s2e::components
