/**
 * @file obc_communication_base.hpp
 * @brief Base class for serial communication (e.g., UART) with OBC flight software
 */

#ifndef S2E_COMPONENTS_BASE_CLASSES_OBC_COMMUNICATION_BASE_HPP_
#define S2E_COMPONENTS_BASE_CLASSES_OBC_COMMUNICATION_BASE_HPP_

#include <interface/hils/hils_port_manager.hpp>

#include "../cdh/obc.hpp"

/**
 * @enum OBC_COM_UART_MODE
 * @brief Simulation mode (SILS or HILS)
 * @details In the SILS mode, S2E does not need to communicate with OBC in S2E
 */
enum class OBC_COM_UART_MODE {
  SILS,        //!< Software In the Loop Simulation
  HILS,        //!< Hardware In the Loop Simulation
  MODE_ERROR,  //!< Error
};

/**
 * @class ObcCommunicationBase
 * @brief Base class for serial communication (e.g., UART) with OBC flight software
 * @note Components which want to communicate with OBC using serial communication have to inherit this.
 */
class ObcCommunicationBase {
 public:
  /**
   * @fn ObcCommunicationBase
   * @brief Constructor for SILS mode
   * @note Default buffer size is used
   * @param [in] sils_port_id: Port ID for communication line b/w OBC in the SILS mode
   * @param [in] obc: The communication target OBC
   */
  ObcCommunicationBase(const int sils_port_id, OBC* obc);
  /**
   * @fn ObcCommunicationBase
   * @brief Constructor for SILS mode
   * @param [in] sils_port_id: Port ID for communication line b/w OBC in the SILS mode
   * @param [in] tx_buf_size: TX (Component to OBC) buffer size
   * @param [in] rx_buf_size: RX (OBC to Component) buffer size
   * @param [in] obc: The communication target OBC
   */
  ObcCommunicationBase(const int sils_port_id, const int tx_buf_size, const int rx_buf_size, OBC* obc);
  /**
   * @fn ObcCommunicationBase
   * @brief Constructor for HILS mode
   * @note Default buffer size is used
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] baud_rate: Baud rate of HILS communication port
   * @param [in] hils_port_manager: HILS port manager
   */
  ObcCommunicationBase(const unsigned int hils_port_id, const unsigned int baud_rate, HilsPortManager* hils_port_manager);
  /**
   * @fn ObcCommunicationBase
   * @brief Constructor for HILS mode
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] baud_rate: Baud rate of HILS communication port
   * @param [in] tx_buf_size: TX (Component to OBC) buffer size
   * @param [in] rx_buf_size: RX (OBC to Component) buffer size
   * @param [in] hils_port_manager: HILS port manager
   */
  ObcCommunicationBase(const unsigned int hils_port_id, const unsigned int baud_rate, const int tx_buf_size, const int rx_buf_size,
                       HilsPortManager* hils_port_manager);
  /**
   * @fn ObcCommunicationBase
   * @brief Constructor for both SILS and HILS mode
   * @note Default buffer size is used
   * @param [in] sils_port_id: Port ID for communication line b/w OBC in the SILS mode
   * @param [in] obc: The communication target OBC
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] baud_rate: Baud rate of HILS communication port
   * @param [in] hils_port_manager: HILS port manager
   */
  ObcCommunicationBase(const int sils_port_id, OBC* obc, const unsigned int hils_port_id, const unsigned int baud_rate,
                       HilsPortManager* hils_port_manager);
  /**
   * @fn ~ObcCommunicationBase
   * @brief Destructor
   */
  ~ObcCommunicationBase();
  /**
   * @fn IsConnected
   * @brief Return connection flag
   */
  inline bool IsConnected() const { return is_connected_; }

 protected:
  int ReceiveCommand(const int offset, const int rec_size);
  int SendTelemetry(const int offset);
  std::vector<unsigned char> tx_buffer_;
  std::vector<unsigned char> rx_buffer_;

 private:
  const int kDefaultBufferSize = 1024;  //!< Default buffer size Fixme: The magic number. This is depending on uart_port.hpp.

  int sils_port_id_;           //!< Port ID for SILS
  int hils_port_id_;           //!< Port ID for HILS
  int baud_rate_;              //!< Baudrate for HILS ex. 9600, 115200
  int tx_buf_size_;            //!< TX (Component to OBC) buffer size
  int rx_buf_size_;            //!< RX (OBC to Component) buffer size
  bool is_connected_ = false;  // Connection flag

  OBC_COM_UART_MODE sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;  //!< Simulation mode

  OBC* obc_;                            //!< Communication target OBC
  HilsPortManager* hils_port_manager_;  //!< HILS port manager

  /**
   * @fn InitializeObcComBase
   * @brief Initialize function
   */
  void InitializeObcComBase();
  /**
   * @fn ParseCommand
   * @brief Pure virtual function for parse command feature
   * @return Error code (ret<=0 means error, ret>0 means fine)
   */
  virtual int ParseCommand(const int cmd_size) = 0;
  /**
   * @fn GenerateTelemetry
   * @brief Pure virtual function for generate telemetry feature
   * @return Telemetry size
   */
  virtual int GenerateTelemetry() = 0;
};

#endif  // S2E_COMPONENTS_BASE_CLASSES_OBC_COMMUNICATION_BASE_HPP_
