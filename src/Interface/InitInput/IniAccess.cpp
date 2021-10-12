#include "IniAccess.h"
#include <string.h>

using namespace std;

IniAccess::IniAccess(string path) :
file_path_(path)
{
  //読み出しファイル名の取得
  // TODO: Modify the codes for multi-platform support
  //strcpy_s(strPath_, (size_t)_countof(strPath_), file_path_.c_str());
  strncpy(strPath_, file_path_.c_str(), MAX_PATH);
}

double IniAccess::ReadDouble(const char* section_name, const char* key_name)
{
  stringstream value;
  double temp = 0;

  GetPrivateProfileStringA(
    section_name,
    key_name,
    0, strText_, 1024, strPath_);

  value << strText_;	//文字列を入力
  value >> temp;		//doubleで返却
  //	cout << strText_;

  return temp;
}

int IniAccess::ReadInt(const char* section_name, const char* key_name)
{
  int temp;

  temp = GetPrivateProfileIntA(
    section_name,
    key_name,
    0, strPath_);

  return temp;
}
bool IniAccess::ReadBoolean(const char* section_name, const char* key_name)
{
  int temp;

  temp = GetPrivateProfileIntA(
    section_name,
    key_name,
    0, strPath_);
  if (temp > 0){ return true; }
  return false;
}

void IniAccess::ReadDoubleArray(const char* section_name, const char* key_name, int id, int num, double* data)
{
  for (int i = 0; i < num; i++){
    stringstream c_name;
    c_name << key_name << id << "(" << i << ")";
    data[i] = ReadDouble(section_name, c_name.str().c_str());
  }
}

void IniAccess::ReadQuaternion(const char* section_name, const char* key_name, Quaternion& data)
{
  Quaternion temp;
  double norm = 0.0;

  for (int i = 0; i < 4; i++){		// 新しいフォーマットに沿ってQuaternionを読み込む
    stringstream c_name;
    c_name << key_name << "_(" << i << ")";
    temp[i] = ReadDouble(section_name, c_name.str().c_str());
    norm += temp[i] * temp[i];
  }
  if (norm == 0.0){			// 新しいフォーマットが読み込めなかった（古いConfigファイルの場合）
    for (int i = 0; i < 4; i++){
      stringstream c_name;
      c_name << key_name << "(" << i << ")";
      data[i] = ReadDouble(section_name, c_name.str().c_str());
    }
  }
  else{						// 新しいフォーマットが読み込めた場合（新しいConfigファイルの場合）
    data[0] = temp[0];
    data[1] = temp[1];
    data[2] = temp[2];
    data[3] = temp[3];
  }
}

void IniAccess::ReadChar(const char* section_name, const char* key_name, int size, char* data)
{
  GetPrivateProfileStringA(
    section_name,
    key_name,
    0, data, size, strPath_);
}

string IniAccess::ReadString(const char * section_name, const char * key_name)
{
  char temp[1024];
  ReadChar(section_name, key_name, 1024, temp);
  return string(temp);
}

bool IniAccess::ReadEnable(const char * section_name, const char * key_name)
{
  string enablestr = ReadString(section_name, key_name);
  if (enablestr.compare("ENABLE") == 0) return true;
  if (enablestr.compare("1") == 0) return true;
  return false;
}

vector<string> IniAccess::ReadStrVector(const char* section_name, const char* key_name)
{
  const static unsigned int buf_size = 1024;
  vector<string> data;
  char temp[buf_size];
  unsigned int i = 0;
  while (true)
  {
    stringstream c_name;
    c_name << key_name << "(" << i << ")";
    ReadChar(section_name, c_name.str().c_str(), buf_size, temp);
    if (temp[0] == NULL)
    {
      break;
    }
    else
    {
      data.push_back(temp);
      i++;
    }
  }
  return data;
}

vector<string> IniAccess::Split(string& input, char delimiter)
{
	istringstream stream(input);
	string field;
	vector<string> result;
	while (getline(stream, field, delimiter)) {
		result.push_back(field);
	}
	return result;
}

void IniAccess::ReadCsvDouble(vector<vector<double>>& doublevec,int node_num)
{
	ifstream ifs(strPath_);
  if (!ifs.is_open())
  {
    cerr << "file open error. filename = " << strPath_ << std::endl;
  }
	string line;
	int line_num = 0;
	doublevec.reserve(node_num);
	while (getline(ifs, line)) {
		vector<string> strvec = Split(line, ',');
		vector<double> tempdoublevec;
		tempdoublevec.reserve(node_num);
		for (int i = 0; i<strvec.size(); i++) {
			tempdoublevec.push_back(stod(strvec.at(i)));
		}
		doublevec.push_back(tempdoublevec);
		line_num++;
	}
}

void IniAccess::ReadCsvString(vector<vector<string>>& stringvec,int node_num)
{
	ifstream ifs(strPath_);
	string line;
	int line_num = 0;
	stringvec.reserve(node_num);
	while (getline(ifs, line)) {
		vector<string> tempstrvec = Split(line, ',');
		tempstrvec.reserve(node_num);
		stringvec.push_back(tempstrvec);
		line_num++;
	}
}
