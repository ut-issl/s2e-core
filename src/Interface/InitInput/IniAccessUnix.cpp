#include "IniAccess.h"
#include <unistd.h>
#include <limits>
#include <algorithm>
#include <cstring>

using namespace std;
static double inf = std::numeric_limits<double>::infinity();

IniAccess::IniAccess(string path) :
file_path_(path), reader(path)
{
  if (path.substr(3) == "ini"){
    if(reader.ParseError() != 0) {
        cout << "Error reading INI file : " << path << endl;
        cout << "\t error code: " << reader.ParseError() << endl;
        throw "Error reading INI file";
    }
  }
}

double IniAccess::ReadDouble(const char* section_name, const char* key_name)
{
  return reader.GetReal(section_name, key_name, 0);	//文字列を入力
}

int IniAccess::ReadInt(const char* section_name, const char* key_name)
{
  return (int)reader.GetInteger(section_name, key_name, 0);
}
bool IniAccess::ReadBoolean(const char* section_name, const char* key_name)
{
  return reader.GetBoolean(section_name, key_name, false);
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
    string sdata = ReadString(section_name, key_name);
    strncpy(data, sdata.c_str(), size);
}

string IniAccess::ReadString(const char * section_name, const char * key_name)
{
  string value = reader.GetString(section_name, key_name, "NULL");
  return value;

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
    if (!strcmp(temp,"NULL"))
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
	ifstream ifs(file_path_);
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
	ifstream ifs(file_path_);
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
