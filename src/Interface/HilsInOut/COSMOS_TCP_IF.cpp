/**
 * @file COSMOS_TCP_IF.cpp
 * @brief TCP interface to communicate with COSMOS software
 * @details Currently, this feature supports Windows Visual Studio only.(FIXME)
 * @note This file is very old and recently not managed well...
 */

#include "COSMOS_TCP_IF.h"

// References for TCP communication (Written in Japanese)
// https://web.archive.org/web/20181106125907/http://www.geocities.jp/playtown1056/program/index.htm

COSMOS_TCP_IF::COSMOS_TCP_IF() : kPortNum(COSMOS_TCP_PORT_DEFAULT) { is_connected_ = false; }

COSMOS_TCP_IF::COSMOS_TCP_IF(unsigned short port_num, const char *server_ip) : kPortNum(port_num), kServerIP(server_ip) { is_connected_ = false; }

COSMOS_TCP_IF::~COSMOS_TCP_IF() {
  closesocket(sock_);  // Not to forget
  WSACleanup();
}

int COSMOS_TCP_IF::Initialize() {
  int ret;
  // Windows
  WSADATA data;

  // Initialize winsock2
  ret = WSAStartup(MAKEWORD(2, 0), &data);
  if (ret != 0) {
    printf("WSAStartupに失敗しました。\n");
    return ret;
  }

  // Setting socket
  sock_ = socket(AF_INET, SOCK_STREAM, 0);

  // Setting server
  server_.sin_family = AF_INET;
  server_.sin_port = htons(10005);

  // Setting IP address of server as localhost address
  const char *ip = kServerIP.c_str();
  server_.sin_addr.s_addr = inet_addr(ip);
  if (server_.sin_addr.s_addr == 0xffffffff) {
    struct hostent *host;

    // Get IP from name (named by DSN)
    host = gethostbyname(ip);
    if (host == NULL) {
      return 1;
    }
    // Possible to get 32 bit IP from the host
    server_.sin_addr.s_addr = *(unsigned int *)host->h_addr_list[0];
  }

  // Connect server
  ret = connect(sock_, (struct sockaddr *)&server_, sizeof(server_));
  if (ret != 0) {
    printf("COSMOSサーバーとの接続に失敗しました。\n");
    return ret;
  } else {
    // Success to connect
    is_connected_ = true;
    printf("COSMOSサーバーとの接続に成功しました\n");
    return 0;
  }
}

void COSMOS_TCP_IF::SendString(std::string str) {
  if (!is_connected_) return;

  std::string ret = str + ENDOFMSG;
  // Send string to COSMOS
  send(sock_, ret.c_str(), ret.length(), 0);
}

void COSMOS_TCP_IF::Finalize() {
  closesocket(sock_);
  is_connected_ = false;
}
