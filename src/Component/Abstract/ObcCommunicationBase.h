#pragma once
#include "../../Interface/HilsInOut/HilsPortManager.h"
#include "../CDH/OBC.h"

enum class OBC_COM_UART_MODE {
  SILS,
  HILS,
  MODE_ERROR,
};

class ObcCommunicationBase {
public:
  ObcCommunicationBase(const int sils_port_id, OBC *obc);
  ObcCommunicationBase(const int sils_port_id, const int tx_buf_size,
                       const int rx_buf_size, OBC *obc);
  ObcCommunicationBase(const unsigned int hils_port_id,
                       const unsigned int baud_rate,
                       HilsPortManager *hils_port_manager);
  ObcCommunicationBase(const unsigned int hils_port_id,
                       const unsigned int baud_rate, const int tx_buf_size,
                       const int rx_buf_size,
                       HilsPortManager *hils_port_manager);
  ObcCommunicationBase(const int sils_port_id, OBC *obc,
                       const unsigned int hils_port_id,
                       const unsigned int baud_rate,
                       HilsPortManager *hils_port_manager);
  ~ObcCommunicationBase();

  inline const bool IsConnected() const { return is_connected_; }

protected:
  int ReceiveCommand(const int offset, const int rec_size);
  int SendTelemetry(const int offset);
  std::vector<unsigned char> tx_buffer_;
  std::vector<unsigned char> rx_buffer_;

private:
  const int kDefaultBufferSize =
      1024; // Fixme: The magic number. This is depending on SCIPort.h.
  int sils_port_id_;
  int hils_port_id_;
  int baud_rate_; // [baud] ex. 9600, 115200
  int tx_buf_size_;
  int rx_buf_size_;
  bool is_connected_ = false;
  OBC_COM_UART_MODE sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;

  OBC *obc_;
  HilsPortManager *hils_port_manager_;

  void InitializeObcComBase();
  virtual int ParseCommand(
      const int cmd_size) = 0; // return value should be error code (ret<=0
                               // means error, ret>0 means fine)
  virtual int
  GenerateTelemetry() = 0; // return value should be the telemetry size
};
