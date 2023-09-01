/**
 * @file c2a_command_database.cpp
 * @brief Classes and functions to read and manage C2A command database
 */

#include "c2a_command_database.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

#ifdef USE_C2A
#if C2A_CORE_VER_MAJOR == 4
// c2a-core v4
#include "src_core/library/endian.h"
#elif C2A_CORE_VER_MAJOR <= 3
// c2a-core <= v3
#include "src_core/Library/endian.h"
#else
#error "c2a-core version is not supported"
#endif  // c2a-core version header
#endif  // USE_C2A

C2aCommandInformation::C2aCommandInformation(const std::string cmd_db_line) {
  if (cmd_db_line.find("*") == 0) return;

  // CSV
  std::string token;
  std::vector<std::string> tokens;
  std::istringstream token_stream(cmd_db_line);
  while (std::getline(token_stream, token, ',')) {
    tokens.push_back(token);
  }

  // Command name
  if (tokens[1].find("Cmd_") != 0) return;
  command_name_ = tokens[1];

  // Command ID
  command_id_ = std::stoul(tokens[3], nullptr, 0);

  // Arguments
  number_of_arguments_ = std::stoul(tokens[4], nullptr, 0);
  for (size_t arg_id = 0; arg_id < number_of_arguments_; arg_id++) {
    std::string argment_type_name = tokens[5 + 2 * arg_id];
    argument_type_info_.push_back(ConvertArgumentType(argment_type_name));
  }
}

C2aArgumentType C2aCommandInformation::GetArgumentType(const size_t argument_id) {
  if (argument_id >= argument_type_info_.size()) {
    return C2aArgumentType::kError;
  } else {
    return argument_type_info_[argument_id];
  }
}

C2aArgumentType C2aCommandInformation::ConvertArgumentType(const std::string type) {
  if (type == "uint8_t") {
    return C2aArgumentType::kUint8t;
  } else if (type == "uint16_t") {
    return C2aArgumentType::kUint16t;
  } else if (type == "uint32_t") {
    return C2aArgumentType::kUint32t;
  } else if (type == "uint64_t") {
    return C2aArgumentType::kUint64t;
  } else if (type == "int8_t") {
    return C2aArgumentType::kInt8t;
  } else if (type == "int16_t") {
    return C2aArgumentType::kInt16t;
  } else if (type == "int32_t") {
    return C2aArgumentType::kInt32t;
  } else if (type == "int64_t") {
    return C2aArgumentType::kInt64t;
  } else if (type == "float") {
    return C2aArgumentType::kFloat;
  } else if (type == "double") {
    return C2aArgumentType::kDouble;
  } else if (type == "raw") {
    return C2aArgumentType::kRaw;
  } else {
    return C2aArgumentType::kError;
  }
}

C2aCommandDatabase::C2aCommandDatabase(const std::string file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "C2A Command DB open error." << std::endl;
  }
  std::string line;
  while (std::getline(file, line)) {
    // コマンド追加
    C2aCommandInformation command(line);
    if (command.GetCommandName() == "Error") continue;
    command_map_[command.GetCommandName()] = command;
  }
}

void DecodeC2aCommandArgument(const C2aArgumentType type, const std::string argument_string, uint8_t* param, size_t& size_param) {
#ifdef USE_C2A
  switch (type) {
    case C2aArgumentType::kUint8t: {
      size_param = 1;
      uint8_t argument = (uint8_t)std::stoul(argument_string);  // TODO: 範囲外処理
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kUint16t: {
      size_param = 2;
      uint16_t argument = (uint16_t)std::stoul(argument_string);  // TODO: 範囲外処理
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kUint32t: {
      size_param = 4;
      uint32_t argument = (uint32_t)std::stoul(argument_string);  // TODO: 範囲外処理
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kUint64t: {
      size_param = 8;
      uint64_t argument = std::stoul(argument_string);
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kInt8t: {
      size_param = 1;
      int8_t argument = (int8_t)std::stoi(argument_string);  // TODO: 範囲外処理
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kInt16t: {
      size_param = 2;
      int16_t argument = (int16_t)std::stoi(argument_string);  // TODO: 範囲外処理
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kInt32t: {
      size_param = 4;
      int32_t argument = std::stoi(argument_string);
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kInt64t: {
      size_param = 8;
      int64_t argument = std::stol(argument_string);
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kFloat: {
      size_param = 4;
      float argument = std::stof(argument_string);
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kDouble: {
      size_param = 8;
      double argument = std::stod(argument_string);
      ENDIAN_memcpy(param, &argument, size_param);
      break;
    }
    case C2aArgumentType::kRaw: {
      // TODO: Rawの実装
      break;
    }
    default:
      break;
  }
#else
  UNUSED(argument_string);
  UNUSED(param);
  UNUSED(size_param);
#endif
}
