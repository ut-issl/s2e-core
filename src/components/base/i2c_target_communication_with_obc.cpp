/**
 * @file i2c_target_communication_with_obc.cpp
 * @brief Base class for I2C communication as target side with OBC flight software
 */
#include "i2c_target_communication_with_obc.hpp"

#include <iostream>

I2cTargetCommunicationWithObc::I2cTargetCommunicationWithObc(const unsigned int sils_port_id, const unsigned char i2c_address, OBC* obc)
    : sils_port_id_(sils_port_id), i2c_address_(i2c_address), obc_(obc) {
#ifdef USE_HILS
  simulation_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:ON Check compo initialization\n");
#else
  simulation_mode_ = OBC_COM_UART_MODE::SILS;
  obc_->I2cConnectPort(sils_port_id_, i2c_address_);
#endif
}

I2cTargetCommunicationWithObc::I2cTargetCommunicationWithObc(const unsigned int hils_port_id, const unsigned char i2c_address,
                                                             HilsPortManager* hils_port_manager)
    : hils_port_id_(hils_port_id), i2c_address_(i2c_address), hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  simulation_mode_ = OBC_COM_UART_MODE::HILS;
  int ret = hils_port_manager_->I2cTargetConnectComPort(hils_port_id_);
  if (ret != 0) {
    std::cout << "Error: ObcI2cTargetCommunication ConnectComPort ID:" << hils_port_id_ << "\n";
  }
#else
  simulation_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:OFF Check compo initialization\n");
#endif
}

I2cTargetCommunicationWithObc::I2cTargetCommunicationWithObc(const unsigned int sils_port_id, const unsigned int hils_port_id,
                                                             const unsigned char i2c_address, OBC* obc, HilsPortManager* hils_port_manager)
    : sils_port_id_(sils_port_id), hils_port_id_(hils_port_id), i2c_address_(i2c_address), obc_(obc), hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  simulation_mode_ = OBC_COM_UART_MODE::HILS;
  int ret = hils_port_manager_->I2cTargetConnectComPort(hils_port_id_);
  if (ret != 0) {
    std::cout << "Error: ObcI2cTargetCommunication ConnectComPort ID:" << hils_port_id_ << "\n";
  }
#else
  simulation_mode_ = OBC_COM_UART_MODE::SILS;
  obc_->I2cConnectPort(sils_port_id_, i2c_address_);
#endif
}

I2cTargetCommunicationWithObc::I2cTargetCommunicationWithObc(I2cTargetCommunicationWithObc&& object) noexcept
    : sils_port_id_(object.sils_port_id_),
      hils_port_id_(object.hils_port_id_),
      i2c_address_(object.i2c_address_),
      simulation_mode_(object.simulation_mode_),
      obc_(object.obc_),
      hils_port_manager_(object.hils_port_manager_) {
  object.is_moved_ = true;
  object.obc_ = nullptr;
  object.hils_port_manager_ = nullptr;
}

I2cTargetCommunicationWithObc::~I2cTargetCommunicationWithObc() {
  if (is_moved_ == true) return;  // prevent double freeing of memory

  int ret;
  switch (simulation_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      break;
    case OBC_COM_UART_MODE::SILS:
      ret = obc_->I2cCloseComPort(sils_port_id_);
      if (ret != 0) {
        // TODO: Add a flag to select whether to show or hide warnings
        // std::cout << "Already closed or not used: ObcI2cTargetCommunication "
        //              "CloseComPort ID:" << sils_port_id_ << "\n";
      }
      break;
    case OBC_COM_UART_MODE::HILS:
      ret = hils_port_manager_->I2cTargetCloseComPort(hils_port_id_);
      if (ret != 0) {
        // TODO: Add a flag to select whether to show or hide warnings
        // std::cout << "Already closed or not used: ObcI2cTargetCommunication "
        //              "CloseComPort ID:" << hils_port_id_ << "\n";
      }
      break;
    default:
      // NOT REACHED
      break;
  }
}

void I2cTargetCommunicationWithObc::ReadRegister(const unsigned char register_address, unsigned char* data, const unsigned char length) {
  switch (simulation_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      break;
    case OBC_COM_UART_MODE::SILS:
      obc_->I2cComponentReadRegister(sils_port_id_, i2c_address_, register_address, data, length);
      break;
    case OBC_COM_UART_MODE::HILS:
      hils_port_manager_->I2cTargetReadRegister(hils_port_id_, register_address, data, length);
      break;
    default:
      // NOT REACHED
      break;
  }
}

void I2cTargetCommunicationWithObc::WriteRegister(const unsigned char register_address, const unsigned char* data, const unsigned char length) {
  switch (simulation_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      break;
    case OBC_COM_UART_MODE::SILS:
      obc_->I2cComponentWriteRegister(sils_port_id_, i2c_address_, register_address, data, length);
      break;
    case OBC_COM_UART_MODE::HILS:
      hils_port_manager_->I2cTargetWriteRegister(hils_port_id_, register_address, data, length);
      break;
    default:
      // NOT REACHED
      break;
  }
}

void I2cTargetCommunicationWithObc::ReadCommand(unsigned char* data, const unsigned char length) {
  switch (simulation_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      break;
    case OBC_COM_UART_MODE::SILS:
      obc_->I2cComponentReadCommand(sils_port_id_, i2c_address_, data, length);
      break;
    case OBC_COM_UART_MODE::HILS:
      hils_port_manager_->I2cTargetReadCommand(hils_port_id_, data, length);
      break;
    default:
      // NOT REACHED
      break;
  }
}

int I2cTargetCommunicationWithObc::ReceiveCommand() {
  if (simulation_mode_ != OBC_COM_UART_MODE::HILS) return -1;
  return hils_port_manager_->I2cTargetReceive(hils_port_id_);
}

int I2cTargetCommunicationWithObc::SendTelemetry(const unsigned char length) {
  if (simulation_mode_ != OBC_COM_UART_MODE::HILS) return -1;
  return hils_port_manager_->I2cTargetSend(hils_port_id_, length);
}

int I2cTargetCommunicationWithObc::GetStoredFrameCounter() {
  if (simulation_mode_ != OBC_COM_UART_MODE::HILS) return -1;
  return hils_port_manager_->I2cTargetGetStoredFrameCounter(hils_port_id_);
}

int I2cTargetCommunicationWithObc::StoreTelemetry(const unsigned int stored_frame_num, const unsigned char tlm_size) {
  if (simulation_mode_ != OBC_COM_UART_MODE::HILS) return -1;
  int additional_frame_num = stored_frame_num - GetStoredFrameCounter();
  if (additional_frame_num <= 0) return -1;

  // store telemetry in converter up to stored_frame_num
  for (int i = 0; i < additional_frame_num; i++) {
    SendTelemetry(tlm_size);
  }
  return 0;
}
