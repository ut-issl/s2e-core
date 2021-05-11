#pragma once
#include "../CDH/OBC.h"

class ObcI2cCommunicationBase
{
public:
  ObcI2cCommunicationBase(
    const int port_id, 
    const unsigned char i2c_address,
    OBC* obc
  );
  ~ObcI2cCommunicationBase();

protected:
  void ReadRegister (const unsigned char reg_addr, unsigned char* data, const unsigned char len);
  void WriteRegister(const unsigned char reg_addr, const unsigned char* data, const unsigned char len); 

private:
  int port_id_;
  unsigned char i2c_address_;

  OBC* obc_;

};