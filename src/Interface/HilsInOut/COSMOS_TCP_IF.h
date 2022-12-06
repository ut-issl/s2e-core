/**
 * @file COSMOS_TCP_IF.h
 * @brief TCP interface to communicate with COSMOS software
 * @details Currently, this feature supports Windows Visual Studio only.(FIXME)
 * @note This file is very old and recently not managed well...
 */

#pragma once
#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif
#include <stdio.h>
#include <time.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <string>

#define COSMOS_TCP_PORT_DEFAULT 10001

#define ENDOFMSG "\n"

/**
 * @class COSMOS_TCP_IF
 * @brief TCP interface to communicate with COSMOS software
 */
class COSMOS_TCP_IF {
 public:
  /**
   * @fn COSMOS_TCP_IF
   * @brief Default Constructor.
   */
  COSMOS_TCP_IF();
  /**
   * @fn COSMOS_TCP_IF
   * @brief Constructor
   */
  COSMOS_TCP_IF(unsigned short port_num, const char* server_ip);
  /**
   * @fn ~COSMOS_TCP_IF
   * @brief Destructor
   */
  ~COSMOS_TCP_IF();

  /**
   * @fn Initialize
   * @brief Open and initialize the TCP socket
   */
  int Initialize();
  /**
   * @fn Finalize
   * @brief Close and finalize the TCP socket
   */
  void Finalize();

 protected:
  /**
   * @fn SendString
   * @brief Send string to COSMOS
   */
  void SendString(std::string str);

 private:
  const unsigned short kPortNum;  //!< Port Number
  const std::string kServerIP;    //!< IP address of the server
  SOCKET sock_;                   //!< socket
  // struct sockaddr_in srcAddr_;
  // struct sockaddr_in dstAddr_;
  struct sockaddr_in server_;  //!< Server

  // int dstAddrSize_ = sizeof(dstAddr_);
  int serverAddrSize = sizeof(server_);  //!< Server address size
  bool is_connected_;                    //!< Is connected
};
