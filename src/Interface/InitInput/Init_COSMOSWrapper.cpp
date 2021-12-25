#include "Initialize.h"
#include "../HilsInOut/COSMOSWrapper.h"

COSMOSWrapper* Init_COSMOSWrapper(string file_name)
{
  IniAccess ini_file(file_name);

  char* section = "COSMOS";
  const unsigned int ip_name_len = 16;
  char server_ip[ip_name_len];
  bool connect_to_cosmos = ini_file.ReadEnable(section, "ConnectToCosmos");
  unsigned short cosmos_tcp_port_num = ini_file.ReadInt(section, "TCPPortNum");
  ini_file.ReadChar(section, "ServerIP", ip_name_len, server_ip);

  COSMOSWrapper* cosmos_wrapper = new COSMOSWrapper(cosmos_tcp_port_num, connect_to_cosmos, server_ip);
  return cosmos_wrapper;
}