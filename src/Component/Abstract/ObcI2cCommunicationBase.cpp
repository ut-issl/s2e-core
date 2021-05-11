#include "ObcI2cCommunicationBase.h"

ObcI2cCommunicationBase::ObcI2cCommunicationBase(int port_id, const unsigned char i2c_address, OBC* obc)
: port_id_(port_id), i2c_address_(i2c_address), obc_(obc)
{
  obc_->I2cConnectPort(port_id_, i2c_address_);
}

ObcI2cCommunicationBase::~ObcI2cCommunicationBase()
{
  int ret = obc_->CloseComPort(port_id_);
}

void ObcI2cCommunicationBase::ReadRegister (const unsigned char reg_addr, unsigned char* data, const unsigned char len)
{
  obc_->I2cComponentReadRegister(port_id_, i2c_address_, reg_addr, data, len);
}

void ObcI2cCommunicationBase::WriteRegister(const unsigned char reg_addr, const unsigned char* data, const unsigned char len)
{
  obc_->I2cComponentWriteRegister(port_id_, i2c_address_, reg_addr, data, len);
}