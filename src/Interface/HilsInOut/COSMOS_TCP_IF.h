#pragma once
#ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif
#include <stdio.h>

// For multi-platform support, we have to modify the dependency
// TODO: put these dependencies in a DEFINE
#include <time.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <string>

#define COSMOS_TCP_PORT_DEFAULT 10001

#define ENDOFMSG "\n"

// COSMOSとTCPで通信するクラス
class COSMOS_TCP_IF {
 public:
  COSMOS_TCP_IF();
  COSMOS_TCP_IF(unsigned short port_num, const char* server_ip);
  ~COSMOS_TCP_IF();
  int Initialize();
  void Finalize();

 protected:
  void SendString(std::string str);

 private:
  const unsigned short kPortNum;
  const std::string kServerIP;
  SOCKET sock_;
  // struct sockaddr_in srcAddr_;
  // struct sockaddr_in dstAddr_;
  struct sockaddr_in server_;

  // int dstAddrSize_ = sizeof(dstAddr_);
  int serverAddrSize = sizeof(server_);
  bool is_connected_;
};
