#include <iostream>

#include "ObcCommunicationBase.h"

ObcCommunicationBase::ObcCommunicationBase(int sils_port_id, OBC *obc)
    : sils_port_id_(sils_port_id), obc_(obc) {
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:ON Check compo initialization");
#else
  sim_mode_ = OBC_COM_UART_MODE::SILS;
#endif
  tx_buf_size_ = kDefaultBufferSize;
  rx_buf_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

ObcCommunicationBase::ObcCommunicationBase(int sils_port_id,
                                           const int tx_buf_size,
                                           const int rx_buf_size, OBC *obc)
    : sils_port_id_(sils_port_id), tx_buf_size_(tx_buf_size),
      rx_buf_size_(rx_buf_size), obc_(obc) {
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:ON Check compo initialization");
#else
  sim_mode_ = OBC_COM_UART_MODE::SILS;
#endif
  if (tx_buf_size_ > kDefaultBufferSize)
    tx_buf_size_ = kDefaultBufferSize;
  if (rx_buf_size_ > kDefaultBufferSize)
    rx_buf_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

ObcCommunicationBase::ObcCommunicationBase(const unsigned int hils_port_id,
                                           const unsigned int baud_rate,
                                           HilsPortManager *hils_port_manager)
    : hils_port_id_(hils_port_id), baud_rate_(baud_rate),
      hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::HILS;
#else
  sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:OFF Check compo initialization");
#endif
  tx_buf_size_ = kDefaultBufferSize;
  rx_buf_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

ObcCommunicationBase::ObcCommunicationBase(const unsigned int hils_port_id,
                                           const unsigned int baud_rate,
                                           const int tx_buf_size,
                                           const int rx_buf_size,
                                           HilsPortManager *hils_port_manager)
    : hils_port_id_(hils_port_id), baud_rate_(baud_rate),
      tx_buf_size_(tx_buf_size), rx_buf_size_(rx_buf_size),
      hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::HILS;
#else
  sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:OFF Check compo initialization");
#endif
  if (tx_buf_size_ > kDefaultBufferSize)
    tx_buf_size_ = kDefaultBufferSize;
  if (rx_buf_size_ > kDefaultBufferSize)
    rx_buf_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

ObcCommunicationBase::ObcCommunicationBase(const int sils_port_id, OBC *obc,
                                           const unsigned int hils_port_id,
                                           const unsigned int baud_rate,
                                           HilsPortManager *hils_port_manager)
    : sils_port_id_(sils_port_id), obc_(obc), hils_port_id_(hils_port_id),
      baud_rate_(baud_rate), hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  sim_mode_ = OBC_COM_UART_MODE::HILS;
#else
  sim_mode_ = OBC_COM_UART_MODE::SILS;
#endif
  tx_buf_size_ = kDefaultBufferSize;
  rx_buf_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

ObcCommunicationBase::~ObcCommunicationBase() {
  if (is_connected_ == false)
    return;

  if (sim_mode_ == OBC_COM_UART_MODE::MODE_ERROR) {
    return;
  } else if (sim_mode_ == OBC_COM_UART_MODE::SILS) {
    int ret = obc_->CloseComPort(sils_port_id_);
    if (ret != 0) {
      std::cout << "Error: ObcCommunication CloseComPort ID:" << sils_port_id_
                << "\n";
    }
  } else // sim_mode_ == OBC_COM_UART_MODE::HILS
  {
    int ret = hils_port_manager_->UartCloseComPort(hils_port_id_);
    if (ret != 0) {
      std::cout << "Error: ObcCommunication CloseComPort ID:" << hils_port_id_
                << "\n";
    }
  }
}

void ObcCommunicationBase::InitializeObcComBase() {
  if (sim_mode_ == OBC_COM_UART_MODE::MODE_ERROR) {
    return;
  } else if (sim_mode_ == OBC_COM_UART_MODE::SILS) {
    int ret = obc_->ConnectComPort(sils_port_id_, tx_buf_size_, rx_buf_size_);
    if (ret != 0) {
      std::cout << "Already connected: ObcCommunication ConnectComPort ID:"
                << sils_port_id_ << "\n";
      is_connected_ = false;
    } else {
      is_connected_ = true;
    }
  } else // sim_mode_ == OBC_COM_UART_MODE::HILS
  {
    int ret = hils_port_manager_->UartConnectComPort(
        hils_port_id_, baud_rate_, tx_buf_size_, rx_buf_size_);
    if (ret != 0) {
      std::cout << "Error: ObcCommunication ConnectComPort ID:" << hils_port_id_
                << "\n";
      is_connected_ = false;
    } else {
      is_connected_ = true;
    }
  }
}

int ObcCommunicationBase::ReceiveCommand(const int offset, const int rec_size) {
  if (sim_mode_ == OBC_COM_UART_MODE::MODE_ERROR)
    return -1;
  if (offset > rx_buf_size_)
    return -1;
  if (offset + rec_size > rx_buf_size_)
    return -1;
  rx_buffer_.resize(rec_size);

  if (sim_mode_ == OBC_COM_UART_MODE::SILS) {
    int ret = obc_->ReceivedByCompo(sils_port_id_, &rx_buffer_.front(), offset,
                                    rec_size);
    if (ret == 0)
      return 0; // No read data
    return ParseCommand(ret);
  } else // sim_mode_ == OBC_COM_UART_MODE::HILS
  {
    int ret = hils_port_manager_->UartReceive(
        hils_port_id_, &rx_buffer_.front(), offset, rec_size);
    if (ret == 0)
      return 0; // No read data
    return ParseCommand(ret);
  }
}
int ObcCommunicationBase::SendTelemetry(const int offset) {
  if (sim_mode_ == OBC_COM_UART_MODE::MODE_ERROR)
    return -1;
  int tlm_size = GenerateTelemetry();
  if (offset > rx_buf_size_)
    return -1;
  if (offset + tlm_size > rx_buf_size_)
    return -1;

  if (sim_mode_ == OBC_COM_UART_MODE::SILS) {
    obc_->SendFromCompo(sils_port_id_, &tx_buffer_.front(), offset, tlm_size);
    return 0;
  } else // sim_mode_ == OBC_COM_UART_MODE::HILS
  {
    hils_port_manager_->UartSend(hils_port_id_, &tx_buffer_.front(), offset,
                                 tlm_size);
    return 0;
  }
}
