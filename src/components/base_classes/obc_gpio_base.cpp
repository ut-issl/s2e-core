/**
 * @file obc_gpio_base.cpp
 * @brief Base class for GPIO communication with OBC flight software
 *        TODO: consider relation with IGPIOCompo
 */

#include "obc_gpio_base.hpp"

ObcGpioBase::ObcGpioBase(const std::vector<int> port_id, OBC* obc) : port_id_(port_id), obc_(obc) {
  for (size_t i = 0; i < port_id_.size(); i++) {
    obc_->GpioConnectPort(port_id_[i]);
  }
}

ObcGpioBase::~ObcGpioBase() {}

bool ObcGpioBase::Read(const int idx) { return obc_->GpioComponentRead(port_id_[idx]); }

void ObcGpioBase::Write(const int idx, const bool is_high) { obc_->GpioComponentWrite(port_id_[idx], is_high); }
