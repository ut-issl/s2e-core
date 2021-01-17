#pragma once
#include "../CDH/OBC.h"

class ObcCommunicationBase
{
public:
  ObcCommunicationBase(
    const int port_id, 
    OBC* obc
  );
  ObcCommunicationBase(
    const int port_id, 
    const int tx_buf_size, 
    const int rx_buf_size, 
    OBC* obc
  );
  ~ObcCommunicationBase();

  inline const bool IsConnected() const{return is_connected_;}

protected:
  int ReceiveCommand(const int offset, const int rec_size);
  int SendTelemetry(const int offset); 
  vector<unsigned char> tx_buffer_;
  vector<unsigned char> rx_buffer_;

private:
  const int kDefaultBufferSize = 1024;// Fixme: The magic number. This is depending on SCIPort.h.
  int port_id_;
  int tx_buf_size_;
  int rx_buf_size_;
  bool is_connected_ = false;

  OBC* obc_;

  void InitializeObcComBase();
  virtual int ParseCommand(const int cmd_size)=0;  // return value should be error code (ret<=0 means error, ret>0 means fine)
  virtual int GenerateTelemetry()=0;  //return value should be the telemetry size
};