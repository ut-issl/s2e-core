#include <iostream>

#include "ObcI2cTargetCommunicationBase.h"

ObcI2cTargetCommunicationBase::ObcI2cTargetCommunicationBase(int sils_port_id, const unsigned char i2c_address, OBC* obc)
: sils_port_id_(sils_port_id), i2c_address_(i2c_address), obc_(obc)
{
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:ON Check compo initialization");
#else
  sim_mode_ = OBC_COM_UART_MODE::SILS;
  obc_->I2cConnectPort(sils_port_id_, i2c_address_);
#endif
}

ObcI2cTargetCommunicationBase::ObcI2cTargetCommunicationBase(const unsigned int hils_port_id, HilsPortManager* hils_port_manager)
: hils_port_id_(hils_port_id), hils_port_manager_(hils_port_manager)
{
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::HILS;
  int ret = hils_port_manager_->I2cConnectComPort(hils_port_id_);
  if (ret != 0)
  {
    std::cout << "Error: ObcI2cCommunication ConnectComPort ID:" << hils_port_id_ << "\n";
  }
#else
  sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:OFF Check compo initialization");
#endif
}

ObcI2cTargetCommunicationBase::ObcI2cTargetCommunicationBase(const int sils_port_id, const unsigned char i2c_address, OBC* obc,
                                                 const unsigned int hils_port_id, HilsPortManager* hils_port_manager)
: sils_port_id_(sils_port_id), i2c_address_(i2c_address), obc_(obc), hils_port_id_(hils_port_id),
  hils_port_manager_(hils_port_manager)
{
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::HILS;
  int ret = hils_port_manager_->I2cConnectComPort(hils_port_id_);
  if (ret != 0)
  {
    std::cout << "Error: ObcI2cCommunication ConnectComPort ID:" << hils_port_id_ << "\n";
  }
#else
  sim_mode_ = OBC_COM_UART_MODE::SILS;
  obc_->I2cConnectPort(sils_port_id_, i2c_address_);
#endif
}

ObcI2cTargetCommunicationBase::~ObcI2cTargetCommunicationBase()
{
  if (sim_mode_ == OBC_COM_UART_MODE::MODE_ERROR) return;

  if (sim_mode_ == OBC_COM_UART_MODE::SILS)
  {
    int ret = obc_->CloseComPort(sils_port_id_);
  }
  else // sim_mode_ == OBC_COM_UART_MODE::HILS
  {
    int ret = hils_port_manager_->I2cCloseComPort(hils_port_id_);
    if (ret != 0)
    {
      std::cout << "Error: ObcI2cCommunication CloseComPort ID:" << hils_port_id_ << "\n";
    }
  }
}

void ObcI2cTargetCommunicationBase::ReadRegister (const unsigned char reg_addr, unsigned char* data, const unsigned char len)
{
  if (sim_mode_ == OBC_COM_UART_MODE::MODE_ERROR) return;

  if (sim_mode_ == OBC_COM_UART_MODE::SILS)
  {
    obc_->I2cComponentReadRegister(sils_port_id_, i2c_address_, reg_addr, data, len);
  }
  else // sim_mode_ == OBC_COM_UART_MODE::HILS
  {
    hils_port_manager_->I2cTargetReadRegister(hils_port_id_, reg_addr, data, len);
  }
}

void ObcI2cTargetCommunicationBase::WriteRegister(const unsigned char reg_addr, const unsigned char* data, const unsigned char len)
{
  if (sim_mode_ == OBC_COM_UART_MODE::MODE_ERROR) return;

  if (sim_mode_ == OBC_COM_UART_MODE::SILS)
  {
    obc_->I2cComponentWriteRegister(sils_port_id_, i2c_address_, reg_addr, data, len);
  }
  else // sim_mode_ == OBC_COM_UART_MODE::HILS
  {
    hils_port_manager_->I2cTargetWriteRegister(hils_port_id_, reg_addr, data, len);
  }
}

void ObcI2cTargetCommunicationBase::ReadCommand(unsigned char* data, const unsigned char len)
{
  if (sim_mode_ == OBC_COM_UART_MODE::MODE_ERROR) return;

  if (sim_mode_ == OBC_COM_UART_MODE::SILS)
  {
    obc_->I2cComponentReadCommand(sils_port_id_, i2c_address_, data, len);
  }
  else // sim_mode_ == OBC_COM_UART_MODE::HILS
  {
    hils_port_manager_->I2cTargetReadCommand(hils_port_id_, data, len);
  }
}

void ObcI2cTargetCommunicationBase::Update() // HILSの際，模擬コンポのMainRoutine()でcallされることを想定
{
  if (sim_mode_ != OBC_COM_UART_MODE::HILS) return;
  hils_port_manager_->I2cTargetUpdateCmd(hils_port_id_);
  hils_port_manager_->I2cTargetUpdateTlm(hils_port_id_);
}
