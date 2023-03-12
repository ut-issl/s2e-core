/**
 * @file uart_communication_with_obc.cpp
 * @brief Base class for serial communication (e.g., UART) with OBC flight software
 */

#include "uart_communication_with_obc.hpp"

#include <iostream>

UartCommunicationWithObc::UartCommunicationWithObc(const unsigned int sils_port_id, OnBoardComputer* obc) : sils_port_id_(sils_port_id), obc_(obc) {
#ifdef USE_HILS
  simulation_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:ON Check compo initialization\n");
#else
  simulation_mode_ = OBC_COM_UART_MODE::SILS;
#endif
  tx_buffer_size_ = kDefaultBufferSize;
  rx_buffer_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

UartCommunicationWithObc::UartCommunicationWithObc(const unsigned int sils_port_id, const unsigned int tx_buffer_size,
                                                   const unsigned int rx_buffer_size, OnBoardComputer* obc)
    : sils_port_id_(sils_port_id), tx_buffer_size_(tx_buffer_size), rx_buffer_size_(rx_buffer_size), obc_(obc) {
#ifdef USE_HILS
  simulation_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:ON Check compo initialization\n");
#else
  simulation_mode_ = OBC_COM_UART_MODE::SILS;
#endif
  if (tx_buffer_size_ > kDefaultBufferSize) tx_buffer_size_ = kDefaultBufferSize;
  if (rx_buffer_size_ > kDefaultBufferSize) rx_buffer_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

UartCommunicationWithObc::UartCommunicationWithObc(const unsigned int hils_port_id, const unsigned int baud_rate, HilsPortManager* hils_port_manager)
    : hils_port_id_(hils_port_id), baud_rate_(baud_rate), hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  simulation_mode_ = OBC_COM_UART_MODE::HILS;
#else
  simulation_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:OFF Check compo initialization\n");
#endif
  tx_buffer_size_ = kDefaultBufferSize;
  rx_buffer_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

UartCommunicationWithObc::UartCommunicationWithObc(const unsigned int hils_port_id, const unsigned int baud_rate, const unsigned int tx_buffer_size,
                                                   const unsigned int rx_buffer_size, HilsPortManager* hils_port_manager)
    : hils_port_id_(hils_port_id),
      baud_rate_(baud_rate),
      tx_buffer_size_(tx_buffer_size),
      rx_buffer_size_(rx_buffer_size),
      hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  simulation_mode_ = OBC_COM_UART_MODE::HILS;
#else
  simulation_mode_ = OBC_COM_UART_MODE::MODE_ERROR;
  printf("Error: USE_HILS:OFF Check compo initialization\n");
#endif
  if (tx_buffer_size_ > kDefaultBufferSize) tx_buffer_size_ = kDefaultBufferSize;
  if (rx_buffer_size_ > kDefaultBufferSize) rx_buffer_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

UartCommunicationWithObc::UartCommunicationWithObc(const int sils_port_id, OnBoardComputer* obc, const unsigned int hils_port_id,
                                                   const unsigned int baud_rate, HilsPortManager* hils_port_manager)
    : sils_port_id_(sils_port_id), hils_port_id_(hils_port_id), baud_rate_(baud_rate), obc_(obc), hils_port_manager_(hils_port_manager) {
#ifdef USE_HILS
  simulation_mode_ = OBC_COM_UART_MODE::HILS;
#else
  simulation_mode_ = OBC_COM_UART_MODE::SILS;
#endif
  tx_buffer_size_ = kDefaultBufferSize;
  rx_buffer_size_ = kDefaultBufferSize;
  InitializeObcComBase();
}

UartCommunicationWithObc::~UartCommunicationWithObc() {
  if (is_connected_ == false) return;
  int ret;
  switch (simulation_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      std::cout << "Error: ObcCommunication CloseComPort MODE_ERROR\n";
      break;
    case OBC_COM_UART_MODE::SILS:
      ret = obc_->CloseComPort(sils_port_id_);
      if (ret != 0) {
        std::cout << "Error: ObcCommunication CloseComPort ID:" << sils_port_id_ << "\n";
      }
      break;
    case OBC_COM_UART_MODE::HILS:
      ret = hils_port_manager_->UartCloseComPort(hils_port_id_);
      if (ret != 0) {
        std::cout << "Error: ObcCommunication CloseComPort ID:" << hils_port_id_ << "\n";
      }
      break;
    default:
      // NOT REACHED
      break;
  }
}

void UartCommunicationWithObc::InitializeObcComBase() {
  int ret;
  switch (simulation_mode_) {
    case OBC_COM_UART_MODE::MODE_ERROR:
      break;
    case OBC_COM_UART_MODE::SILS:
      ret = obc_->ConnectComPort(sils_port_id_, tx_buffer_size_, rx_buffer_size_);
      if (ret != 0) {
        std::cout << "Already connected: ObcCommunication ConnectComPort ID:" << sils_port_id_ << "\n";
        is_connected_ = false;
      } else {
        is_connected_ = true;
      }
      break;
    case OBC_COM_UART_MODE::HILS:
      ret = hils_port_manager_->UartConnectComPort(hils_port_id_, baud_rate_, tx_buffer_size_, rx_buffer_size_);
      if (ret != 0) {
        std::cout << "Error: ObcCommunication ConnectComPort ID:" << hils_port_id_ << "\n";
        is_connected_ = false;
      } else {
        is_connected_ = true;
      }
      break;
    default:
      // NOT REACHED
      break;
  }
}

int UartCommunicationWithObc::ReceiveCommand(const unsigned int offset, const unsigned int rec_size) {
  if (simulation_mode_ == OBC_COM_UART_MODE::MODE_ERROR) return -1;
  if (offset > rx_buffer_size_) return -1;
  if (offset + rec_size > rx_buffer_size_) return -1;
  rx_buffer_.resize(rec_size);

  int ret;
  switch (simulation_mode_) {
    case OBC_COM_UART_MODE::SILS:
      ret = obc_->ReceivedByCompo(sils_port_id_, &rx_buffer_.front(), offset, rec_size);
      if (ret == 0) return 0;  // No read data
      return ParseCommand(ret);
    case OBC_COM_UART_MODE::HILS:
      ret = hils_port_manager_->UartReceive(hils_port_id_, &rx_buffer_.front(), offset, rec_size);
      if (ret == 0) return 0;  // No read data
      return ParseCommand(ret);
    default:
      // NOT REACHED
      return -1;
      ;
  }
}
int UartCommunicationWithObc::SendTelemetry(const unsigned int offset) {
  if (simulation_mode_ == OBC_COM_UART_MODE::MODE_ERROR) return -1;
  int tlm_size = GenerateTelemetry();
  if (offset > rx_buffer_size_) return -1;
  if (offset + tlm_size > rx_buffer_size_) return -1;

  switch (simulation_mode_) {
    case OBC_COM_UART_MODE::SILS:
      obc_->SendFromCompo(sils_port_id_, &tx_buffer_.front(), offset, tlm_size);
      return 0;
    case OBC_COM_UART_MODE::HILS:
      hils_port_manager_->UartSend(hils_port_id_, &tx_buffer_.front(), offset, tlm_size);
      return 0;
    default:
      // NOT REACHED
      return -1;
  }
}
