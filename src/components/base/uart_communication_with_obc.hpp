/**
 * @file uart_communication_with_obc.hpp
 * @brief Base class for serial communication (e.g., UART) with OBC flight software
 */

#ifndef S2E_COMPONENTS_BASE_UART_COMMUNICATION_WITH_OBC_HPP_
#define S2E_COMPONENTS_BASE_UART_COMMUNICATION_WITH_OBC_HPP_

#include <simulation/hils/hils_port_manager.hpp>

#include "../real/cdh/on_board_computer.hpp"

namespace s2e::components {

/**
 * @enum SimulationMode
 * @brief Simulation mode (SILS or HILS)
 * @details In the SILS mode, S2E does not need to communicate with OnBoardComputer in S2E
 */
enum class SimulationMode {
  kSils,   //!< Software In the Loop Simulation
  kHils,   //!< Hardware In the Loop Simulation
  kError,  //!< Error
};

/**
 * @class UartCommunicationWithObc
 * @brief Base class for serial communication (e.g., UART) with OBC flight software
 * @note Components which want to communicate with OnBoardComputer using serial communication have to inherit this.
 */
class UartCommunicationWithObc {
 public:
  /**
   * @fn UartCommunicationWithObc
   * @brief Constructor for SILS mode
   * @note Default buffer size is used
   * @param [in] sils_port_id: Port ID for communication line b/w OnBoardComputer in the SILS mode
   * @param [in] obc: The communication target OBC
   */
  UartCommunicationWithObc(const unsigned int sils_port_id, OnBoardComputer* obc);
  /**
   * @fn UartCommunicationWithObc
   * @brief Constructor for SILS mode
   * @param [in] sils_port_id: Port ID for communication line b/w OnBoardComputer in the SILS mode
   * @param [in] tx_buffer_size: TX (Component to OBC) buffer size
   * @param [in] rx_buffer_size: RX (OBC to Component) buffer size
   * @param [in] obc: The communication target OBC
   */
  UartCommunicationWithObc(const unsigned int sils_port_id, const unsigned int tx_buffer_size, const unsigned int rx_buffer_size,
                           OnBoardComputer* obc);
  /**
   * @fn UartCommunicationWithObc
   * @brief Constructor for HILS mode
   * @note Default buffer size is used
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] baud_rate: Baud rate of HILS communication port
   * @param [in] hils_port_manager: HILS port manager
   */
  UartCommunicationWithObc(const unsigned int hils_port_id, const unsigned int baud_rate, HilsPortManager* hils_port_manager);
  /**
   * @fn UartCommunicationWithObc
   * @brief Constructor for HILS mode
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] baud_rate: Baud rate of HILS communication port
   * @param [in] tx_buffer_size: TX (Component to OBC) buffer size
   * @param [in] rx_buffer_size: RX (OBC to Component) buffer size
   * @param [in] hils_port_manager: HILS port manager
   */
  UartCommunicationWithObc(const unsigned int hils_port_id, const unsigned int baud_rate, const unsigned int tx_buffer_size,
                           const unsigned int rx_buffer_size, HilsPortManager* hils_port_manager);
  /**
   * @fn UartCommunicationWithObc
   * @brief Constructor for both SILS and HILS mode
   * @note Default buffer size is used
   * @param [in] sils_port_id: Port ID for communication line b/w OnBoardComputer in the SILS mode
   * @param [in] obc: The communication target OBC
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] baud_rate: Baud rate of HILS communication port
   * @param [in] hils_port_manager: HILS port manager
   */
  UartCommunicationWithObc(const int sils_port_id, OnBoardComputer* obc, const unsigned int hils_port_id, const unsigned int baud_rate,
                           HilsPortManager* hils_port_manager);
  /**
   * @fn ~UartCommunicationWithObc
   * @brief Destructor
   */
  ~UartCommunicationWithObc();
  /**
   * @fn IsConnected
   * @brief Return connection flag
   */
  inline bool IsConnected() const { return is_connected_; }

 protected:
  int ReceiveCommand(const unsigned int offset, const unsigned int rec_size);
  int SendTelemetry(const unsigned int offset);
  std::vector<unsigned char> tx_buffer_;
  std::vector<unsigned char> rx_buffer_;

 private:
  const unsigned int kDefaultBufferSize = 1024;  //!< Default buffer size Fixme: The magic number. This is depending on uart_port.hpp.

  unsigned int sils_port_id_;    //!< Port ID for SILS
  unsigned int hils_port_id_;    //!< Port ID for HILS
  unsigned int baud_rate_;       //!< Baudrate for HILS ex. 9600, 115200
  unsigned int tx_buffer_size_;  //!< TX (Component to OBC) buffer size
  unsigned int rx_buffer_size_;  //!< RX (OBC to Component) buffer size
  bool is_connected_ = false;    //!< Connection flag

  SimulationMode simulation_mode_ = SimulationMode::kError;  //!< Simulation mode

  OnBoardComputer* obc_;                //!< Communication target OBC
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
  virtual int ParseCommand(const int command_size) = 0;
  /**
   * @fn GenerateTelemetry
   * @brief Pure virtual function for generate telemetry feature
   * @return Telemetry size
   */
  virtual int GenerateTelemetry() = 0;
};

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_BASE_UART_COMMUNICATION_WITH_OBC_HPP_
