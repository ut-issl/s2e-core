#include "ObcI2cTargetCommunicationBase.h"

#include <iostream>

ObcI2cTargetCommunicationBase::ObcI2cTargetCommunicationBase(
    const unsigned int sils_port_id, const unsigned char i2c_address, OBC* obc)
    : sils_port_id_(sils_port_id), i2c_address_(i2c_address), obc_(obc) {
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:ON Check compo initialization");
#else
  sim_mode_ = OBC_COM_UART_MODE::SILS;
  obc_->I2cConnectPort(sils_port_id_, i2c_address_);
#endif
}

ObcI2cTargetCommunicationBase::ObcI2cTargetCommunicationBase(
    const unsigned int hils_port_id, const unsigned char i2c_address,
    HilsPortManager* hils_port_manager)
    : hils_port_id_(hils_port_id), hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::HILS;
  int ret = hils_port_manager_->I2cTargetConnectComPort(hils_port_id_);
  if (ret != 0) {
    std::cout << "Error: ObcI2cCommunication ConnectComPort ID:"
              << hils_port_id_ << "\n";
  }
#else
  sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:OFF Check compo initialization");
#endif
}

ObcI2cTargetCommunicationBase::ObcI2cTargetCommunicationBase(
    const unsigned int sils_port_id, const unsigned int hils_port_id,
    const unsigned char i2c_address, OBC* obc,
    HilsPortManager* hils_port_manager)
    : sils_port_id_(sils_port_id),
      i2c_address_(i2c_address),
      obc_(obc),
      hils_port_id_(hils_port_id),
      hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::HILS;
  int ret = hils_port_manager_->I2cTargetConnectComPort(hils_port_id_);
  if (ret != 0) {
    std::cout << "Error: ObcI2cCommunication ConnectComPort ID:"
              << hils_port_id_ << "\n";
  }
#else
  sim_mode_ = OBC_COM_UART_MODE::SILS;
  obc_->I2cConnectPort(sils_port_id_, i2c_address_);
#endif
}

ObcI2cTargetCommunicationBase::ObcI2cTargetCommunicationBase(
    ObcI2cTargetCommunicationBase&& obj) noexcept
    : sils_port_id_(obj.sils_port_id_),
      hils_port_id_(obj.hils_port_id_),
      i2c_address_(obj.i2c_address_),
      obc_(obj.obc_),
      hils_port_manager_(obj.hils_port_manager_),
      sim_mode_(obj.sim_mode_) {
  obj.is_moved_ = true;
  obj.obc_ = nullptr;
  obj.hils_port_manager_ = nullptr;
}

ObcI2cTargetCommunicationBase::~ObcI2cTargetCommunicationBase() {
  if (is_moved_ == true) return;  // prevent double freeing of memory

  int ret;
  switch (sim_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      break;
    case OBC_COM_UART_MODE::SILS:
      ret = obc_->I2cCloseComPort(sils_port_id_);
      // TODO: Add a flag to select whether to show or hide warnings
      // if(ret != 0) {
      //   std::cout << "Already closed or not used: ObcI2cTargetCommunication "
      //                "CloseComPort ID:"
      //             << sils_port_id_ << "\n";
      // }
      break;
    case OBC_COM_UART_MODE::HILS:
      ret = hils_port_manager_->I2cTargetCloseComPort(hils_port_id_);
      // TODO: Add a flag to select whether to show or hide warnings
      // if (ret != 0) {
      //   std::cout << "Already closed or not used: ObcI2cTargetCommunication "
      //                "CloseComPort ID:"
      //             << hils_port_id_ << "\n";
      // }
      break;
    default:
      // NOT REACHED
      break;
  }
}

void ObcI2cTargetCommunicationBase::ReadRegister(const unsigned char reg_addr,
                                                 unsigned char* data,
                                                 const unsigned char len) {
  switch (sim_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      break;
    case OBC_COM_UART_MODE::SILS:
      obc_->I2cComponentReadRegister(sils_port_id_, i2c_address_, reg_addr,
                                     data, len);
      break;
    case OBC_COM_UART_MODE::HILS:
      hils_port_manager_->I2cTargetReadRegister(hils_port_id_, reg_addr, data,
                                                len);
      break;
    default:
      // NOT REACHED
      break;
  }
}

void ObcI2cTargetCommunicationBase::WriteRegister(const unsigned char reg_addr,
                                                  const unsigned char* data,
                                                  const unsigned char len) {
  switch (sim_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      break;
    case OBC_COM_UART_MODE::SILS:
      obc_->I2cComponentWriteRegister(sils_port_id_, i2c_address_, reg_addr,
                                      data, len);
      break;
    case OBC_COM_UART_MODE::HILS:
      hils_port_manager_->I2cTargetWriteRegister(hils_port_id_, reg_addr, data,
                                                 len);
      break;
    default:
      // NOT REACHED
      break;
  }
}

void ObcI2cTargetCommunicationBase::ReadCommand(unsigned char* data,
                                                const unsigned char len) {
  switch (sim_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      break;
    case OBC_COM_UART_MODE::SILS:
      obc_->I2cComponentReadCommand(sils_port_id_, i2c_address_, data, len);
      break;
    case OBC_COM_UART_MODE::HILS:
      hils_port_manager_->I2cTargetReadCommand(hils_port_id_, data, len);
      break;
    default:
      // NOT REACHED
      break;
  }
}

int ObcI2cTargetCommunicationBase::ReceiveCommand() {
  if (sim_mode_ != OBC_COM_UART_MODE::HILS) return -1;
  return hils_port_manager_->I2cTargetReceive(hils_port_id_);
}

int ObcI2cTargetCommunicationBase::SendTelemetry(const unsigned char len) {
  if (sim_mode_ != OBC_COM_UART_MODE::HILS) return -1;
  return hils_port_manager_->I2cTargetSend(hils_port_id_, len);
}

int ObcI2cTargetCommunicationBase::GetStoredFrameCounter() {
  if (sim_mode_ != OBC_COM_UART_MODE::HILS) return -1;
  return hils_port_manager_->I2cTargetGetStoredFrameCounter(hils_port_id_);
}
