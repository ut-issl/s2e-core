#ifndef __IniAccess_H__
#define __IniAccess_H__
#define _CRT_SECURE_NO_WARNINGS

#ifdef WIN32
	#define _WINSOCKAPI_    // stops windows.h including winsock.h
	#include <windows.h>
	#include <tchar.h>
#else
	#include <Library/inih/cpp/INIReader.h>
#endif
#include <string>
#include <vector>
#include <sstream>
#include <fstream>

#include <Library/math/Vector.hpp>
#include <Library/math/Quaternion.hpp>

#undef  MAX_PATH
#define MAX_PATH 1024

using libra::Vector;
using libra::Quaternion;

class IniAccess
{
  //変数
private:
  std::string file_path_;			//読み込みファイルパス
  char strPath_[MAX_PATH];	//読み込みファイルパス
  char strText_[1024];		//バッファ
#ifndef WIN32
  INIReader reader;
#endif

public:
  IniAccess(std::string path);

  //iniファイル読み込み関数
  double ReadDouble(const char* section_name, const char* key_name);
  int ReadInt(const char* section_name, const char* key_name);
  bool ReadBoolean(const char* section_name, const char* key_name);
  void ReadDoubleArray(const char* section_name, const char* key_name, int id, int num, double* data);
  template< size_t NumElement>
  void ReadVector(const char* section_name, const char* key_name, Vector<NumElement>& data);
  std::vector<std::string> ReadStrVector(const char* section_name, const char* key_name);
  void ReadQuaternion(const char* section_name, const char* key_name, Quaternion& data);
  void ReadChar(const char* section_name, const char* key_name, int size, char* data);
  std::string ReadString(const char* section_name, const char* key_name);
  bool ReadEnable(const char* section_name, const char* key_name);
  // CSV読み込み関数
  std::vector<std::string> Split(std::string& input, char delimiter);
  void ReadCsvDouble(std::vector<std::vector<double>>& doublevec, int node_num); /*nxm行列を読み込む場合, n,mの内大きい方を第二引数に代入する(メモリ事前確保のため)*/
  void ReadCsvString(std::vector<std::vector<std::string>>& stringvec, int node_num);
};

//テンプレートなのでヘッダに書く
template<size_t NumElement>
void IniAccess::ReadVector(const char* section_name, const char* key_name, Vector<NumElement>& data)
{
  for (int i = 0; i < NumElement; i++){
    std::stringstream c_name;
    c_name << key_name << "(" << i << ")";
    data[i] = ReadDouble(section_name, c_name.str().c_str());
  }
}

#endif //__IniAccess_H__
