#pragma once

#include <map>
#include "Ports/SCIPort.h"

/**
 * Serial Communication Interface (independent of physical layer and protocol)
 * @note SCI is probably a Renesas term, but I couldn't come up with a suitable
 * term so I named it for now
 */
class SCIDriver
{
public:

  /**
   * Open the port specified by port_id. Returns -1 if it is already open
   * @param port_id
   * @param tx_buf_size
   * @param rx_buf_size
   * @return
   */
  static int ConnectPort(int port_id, int tx_buf_size = 0, int rx_buf_size = 0);

  /**
   * Close a previously connected port and free resources.
   * @param port_id Port to close
   * @return Returns 1 if ok, -1 if port was not open
   */
  static int ClosePort(int port_id);

  /**
   * Writes count bytes from the offset byte of buffer to the transmission
   * buffer for the simulator
   * @param port_id
   * @param buffer
   * @param offset
   * @param count
   * @return
   */
  static int SendToSim(int port_id, unsigned char* buffer, int offset, int count);

  /**
   * Write only count bytes from the offset byte of buffer to the transmission
   * buffer for the installed software
   * @param port_id
   * @param buffer
   * @param offset
   * @param count
   * @return
   */
  static int SendToSC(int port_id, unsigned char* buffer, int offset, int count);

  /**
   * Write only count bytes from the offset byte of buffer to the transmission
   * buffer for the installed software
   * @param port_id
   * @param buffer
   * @param offset
   * @param count
   * @return
   */
  static int ReceiveFromSim(int port_id, unsigned char* buffer, int offset, int count);

  /**
   * Write only count bytes from the offset byte of buffer to the transmission
   * buffer for the installed software
   * @param port_id
   * @param buffer
   * @param offset
   * @param count
   * @return
   */
  static int ReceiveFromSC(int port_id, unsigned char* buffer, int offset, int count);

private:
  static std::map<int, SCIPort*> ports_;
};

