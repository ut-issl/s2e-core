/**
 * @file initialize_file_access.hpp
 * @brief Class to read and get parameters for the `ini` format file
 */

#ifndef S2E_LIBRARY_INITIALIZE_INITIALIZE_FILE_ACCESS_HPP_
#define S2E_LIBRARY_INITIALIZE_INITIALIZE_FILE_ACCESS_HPP_

#define _CRT_SECURE_NO_WARNINGS

#define INI_CALC_LABEL "calculation"
#define INI_LOG_LABEL "logging"

#ifdef WIN32
#define _WINSOCKAPI_  // stops windows.h including winsock.h
#include <tchar.h>
#define NOMINMAX
#include <windows.h>
#else
#include <library/external/inih/cpp/INIReader.h>
#endif

#include <fstream>
#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>
#include <sstream>
#include <string>
#include <vector>

/**
 * @class IniAccess
 * @brief Class to read and get parameters for the `ini` format file
 */
class IniAccess {
 public:
  /**
   * @fn IniAccess
   * @brief Constructor
   * @param[in] file_path: File path of ini file
   */
  IniAccess(const std::string file_path);

  // Read functions
  /**
   * @fn ReadDouble
   * @brief Read a scalar number as double type
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @return Read number
   */
  double ReadDouble(const char* section_name, const char* key_name);
  /**
   * @fn ReadInt
   * @brief Read a scalar number as integer type
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @return Read number
   */
  int ReadInt(const char* section_name, const char* key_name);
  /**
   * @fn ReadVectorInt
   * @brief Read a vector number as integer type
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @param[in] num: Number of elements of the array
   * @return Read number
   */
  std::vector<int> ReadVectorInt(const char* section_name, const char* key_name, const size_t num);
  /**
   * @fn ReadBoolean
   * @brief Read boolean
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @return Read number
   */
  bool ReadBoolean(const char* section_name, const char* key_name);
  /**
   * @fn ReadDoubleArray
   * @brief Read an array of number as double type
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @param[in] id: ID in key name
   * @param[in] num: Number of elements of the array
   * @param[out] data: Read array data
   */
  void ReadDoubleArray(const char* section_name, const char* key_name, const int id, const int num, double* data);
  /**
   * @fn ReadVector
   * @brief Read Vector type number
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @param[out] data: Read vector type data
   */
  template <size_t NumElement>
  void ReadVector(const char* section_name, const char* key_name, libra::Vector<NumElement>& data);
  /**
   * @fn ReadStrVector
   * @brief Read list of string type
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @return Read list of string
   */
  std::vector<std::string> ReadStrVector(const char* section_name, const char* key_name);
  /**
   * @fn ReadQuaternion
   * @brief Read quaternion
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @param[out] data: Read quaternion data
   */
  void ReadQuaternion(const char* section_name, const char* key_name, libra::Quaternion& data);
  /**
   * @fn ReadChar
   * @brief Read characters data
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @param [in] size: Length of the character
   * @param[out] data: Read character data
   */
  void ReadChar(const char* section_name, const char* key_name, const int size, char* data);
  /**
   * @fn ReadString
   * @brief Read string data
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @return Read string data
   */
  std::string ReadString(const char* section_name, const char* key_name);

  /**
   * @fn ReadEnable
   * @brief Return true when the read value is ENABLE or 1.
   * @param[in] section_name: Section name
   * @param[in] key_name: Key name
   * @return Return true when the read value is ENABLE or 1.
   */
  bool ReadEnable(const char* section_name, const char* key_name);

  // Read CSV functions TODO: Make new class for CSV file
  /**
   * @fn Split
   * @brief Split inputted strings with the delimiter.
   * @param[in] input: Input string
   * @param[in] delimiter: Delimiter to split the string
   * @return List of string splitted by the delimiter
   */
  std::vector<std::string> Split(const std::string& input, const char delimiter);
  /**
   * @fn ReadCsvDouble
   * @brief Read matrix value in CSV file
   * @param[out] output_value: Read double matrix value
   * @param[in] node_num: Number of node. When reading n * m matrix, please substitute bigger number.
   */
  void ReadCsvDouble(std::vector<std::vector<double>>& output_value, const size_t node_num);
  /**
   * @fn ReadCsvDoubleWithHeader
   * @brief Read matrix value in CSV file with header
   * @param[out] output_value: Read double matrix value
   * @param[in] node_num: Number of node. When reading n * m matrix, please substitute bigger number.
   * @param[in] row_header_num: Number of rows of header
   * @param[in] column_header_num: Number of columns of header
   */
  void ReadCsvDoubleWithHeader(std::vector<std::vector<double>>& output_value, const size_t node_num, const size_t row_header_num,
                               const size_t column_header_num);
  /**
   * @fn ReadCsvString
   * @brief Read matrix of string in CSV file
   * @param[out] output_value: Read matrix of string
   * @param[in] node_num: Number of node. When reading n * m matrix, please substitute bigger number.
   */
  void ReadCsvString(std::vector<std::vector<std::string>>& output_value, const size_t node_num);

 private:
  static const size_t kMaxCharLength = 1024;
  std::string file_path_;                //!< File path in string
  char file_path_char_[kMaxCharLength];  //!< File path in char
  char text_buffer_[kMaxCharLength];     //!< buffer
#ifndef WIN32
  INIReader ini_reader_;  //!< ini ini_reader_
#endif
};

template <size_t NumElement>
void IniAccess::ReadVector(const char* section_name, const char* key_name, libra::Vector<NumElement>& data) {
  for (size_t i = 0; i < NumElement; i++) {
    std::stringstream c_name;
    c_name << key_name << "(" << i << ")";
    data[i] = ReadDouble(section_name, c_name.str().c_str());
  }
}

#endif  // S2E_LIBRARY_INITIALIZE_INITIALIZE_FILE_ACCESS_HPP_
