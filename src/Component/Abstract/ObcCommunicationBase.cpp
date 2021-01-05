#include "ObcCommunicationBase.h"

ObcCommunicationBase::ObcCommunicationBase(int port_id, OBC* obc)
: port_id_(port_id), obc_(obc)
{
  tx_buf_size_ = kDefaultBufferSize;
  rx_buf_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

ObcCommunicationBase::ObcCommunicationBase(int port_id, const int tx_buf_size, const int rx_buf_size, OBC* obc)
: port_id_(port_id), tx_buf_size_(tx_buf_size), rx_buf_size_(rx_buf_size), obc_(obc)
{
  if (tx_buf_size_ > kDefaultBufferSize) tx_buf_size_ = kDefaultBufferSize;
  if (rx_buf_size_ > kDefaultBufferSize) rx_buf_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

ObcCommunicationBase::~ObcCommunicationBase()
{
  if(is_connected_ == false) return;
  int ret = obc_->CloseComPort(port_id_);
  if (ret != 0)
  {
    cout << "Error: ObcCommunication CloseComPort ID:" << port_id_ << "\n";
  }
}

void ObcCommunicationBase::InitializeObcComBase()
{
  int ret = obc_->ConnectComPort(port_id_,tx_buf_size_,rx_buf_size_);
  if (ret != 0)
  {
    cout << "Already connected: ObcCommunication ConnectComPort ID:" << port_id_ << "\n";
    is_connected_ = false;
  }
  else
  {
    is_connected_ = true;
  }
}

int ObcCommunicationBase::ReceiveCommand(const int offset, const int rec_size)
{
  if (offset > rx_buf_size_) return -1;
  if (offset+rec_size > rx_buf_size_) return -1;
  rx_buffer_.resize(rec_size);
  int ret = obc_->ReceivedByCompo(port_id_, &rx_buffer_.front(), offset, rec_size);
  if (ret == 0) return 0; //No read data

  return ParseCommand(ret);
}
int ObcCommunicationBase::SendTelemetry(const int offset)
{
  int tlm_size = GenerateTelemetry();
  if (offset > rx_buf_size_) return -1;
  if (offset+tlm_size > rx_buf_size_) return -1;
  obc_->SendFromCompo(port_id_, &tx_buffer_.front(), offset, tlm_size);
  return 0;
}