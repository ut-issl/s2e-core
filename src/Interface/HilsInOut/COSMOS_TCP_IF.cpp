#include "COSMOS_TCP_IF.h"

// TCP通信に関しての参考URL
// - http://www.geocities.jp/playtown1056/program/index.htm //
// 2019/3/31以降閲覧不可能
// -
// https://web.archive.org/web/20181106125907/http://www.geocities.jp/playtown1056/program/index.htm
// // 上記Webページのアーカイブ
// - https://www.sbcr.jp/books/img/Linuxnet_02.pdf

COSMOS_TCP_IF::COSMOS_TCP_IF() : kPortNum(COSMOS_TCP_PORT_DEFAULT) {
  is_connected_ = false;
}

COSMOS_TCP_IF::COSMOS_TCP_IF(unsigned short port_num, const char *server_ip)
    : kPortNum(port_num), kServerIP(server_ip) {

  is_connected_ = false;
}

COSMOS_TCP_IF::~COSMOS_TCP_IF() {
  closesocket(sock_); // 忘れないように一応ここでも。
  WSACleanup();
}

int COSMOS_TCP_IF::Initialize() {
  int ret;
  // Windowsの場合
  WSADATA data;

  // winsock2の初期化
  ret = WSAStartup(MAKEWORD(2, 0), &data);
  if (ret != 0) {
    printf("WSAStartupに失敗しました。\n");
    return ret;
  }

  // ソケット設定
  sock_ = socket(AF_INET, SOCK_STREAM, 0);

  // サーバー側の設定
  server_.sin_family = AF_INET;
  server_.sin_port = htons(10005);

  // サーバーのIPアドレスをlocalhost のアドレスに設定
  const char *ip = kServerIP.c_str();
  server_.sin_addr.s_addr = inet_addr(ip);
  if (server_.sin_addr.s_addr == 0xffffffff) {
    struct hostent *host;

    // 名前( DNS がつけた名前 )から IP をひく
    host = gethostbyname(ip);
    if (host == NULL) {
      return 1;
    }
    // host から IP( 32 bit )を取得可能.
    server_.sin_addr.s_addr = *(unsigned int *)host->h_addr_list[0];
  }

  // サーバーに接続
  ret = connect(sock_, (struct sockaddr *)&server_, sizeof(server_));
  if (ret != 0) {
    printf("COSMOSサーバーとの接続に失敗しました。\n");
    return ret;
  } else {
    // 接続できた場合
    is_connected_ = true;
    printf("COSMOSサーバーとの接続に成功しました\n");
    return 0;
  }
}

void COSMOS_TCP_IF::SendString(std::string str) {
  if (!is_connected_)
    return;

  std::string ret = str + ENDOFMSG;
  // COSMOSに文字列を送信する。
  send(sock_, ret.c_str(), ret.length(), 0);
}

void COSMOS_TCP_IF::Finalize() {
  closesocket(sock_);
  is_connected_ = false;
}
