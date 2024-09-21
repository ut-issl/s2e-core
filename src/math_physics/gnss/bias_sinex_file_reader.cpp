/**
 * @file bias_sinex_file_reader.cpp
 * @brief Read bias SINEX format file
 * @note Ref. https://files.igs.org/pub/data/format/sinex_bias_100.pdf
 */

#include "bias_sinex_file_reader.hpp"

#include <fstream>
#include <iostream>

namespace gnss {

bool BiasSinexFileReader::ReadFile(const std::string file_name) {
  // File open
  std::ifstream bias_sinex_file(file_name);
  if (!bias_sinex_file.is_open()) {
    std::cout << "[Warning] Bias SINEX file not found: " << file_name << std::endl;
    return false;
  }

  while (bias_sinex_file.peek() != EOF) {
    std::string line;
    std::getline(bias_sinex_file, line);
    // Skip comments
    if (line.find("*") == 0) {
      continue;
    }
    // Footer
    if (line.find("%=ENDBIA") == 0) {
      break;
    }
    // Header
    if (line.find("%=BIA") == 0) {
      // TODO: read header information
      continue;
    }
    // File/Reference
    if (line.find("+FILE/REFERENCE") == 0) {
      ReadFileReference(bias_sinex_file);
      continue;
    }
    // File/Comment
    if (line.find("+FILE/COMMENT") == 0) {
      ReadFileComment(bias_sinex_file);
      continue;
    }
    // Input/Acknowledgments
    if (line.find("+INPUT/ACKNOWLEDGMENTS") == 0) {
      ReadInputAcknowledgments(bias_sinex_file);
      continue;
    }
    // Bias/Description
    if (line.find("+BIAS/DESCRIPTION") == 0) {
      ReadBiasDescription(bias_sinex_file);
      continue;
    }
    // Bias/ReceiverInformation
    if (line.find("+BIAS/RECEIVER_INFORMATION") == 0) {
      ReadBiasReceiverInformation(bias_sinex_file);
      continue;
    }
    // Bias/Solution
    if (line.find("+BIAS/SOLUTION") == 0) {
      ReadBiasSolution(bias_sinex_file);
      continue;
    }
  }

  // Read epoch wise data
  bias_sinex_file.close();
  return true;
}

void BiasSinexFileReader::ReadFileReference(std::ifstream& bias_sinex_file) {
  std::string line;
  while (1) {
    std::getline(bias_sinex_file, line);
    // End of data
    if (line.find("-FILE/REFERENCE") == 0) {
      break;
    }
    // TODO: read information
  }
}

void BiasSinexFileReader::ReadFileComment(std::ifstream& bias_sinex_file) {
  std::string line;
  while (1) {
    std::getline(bias_sinex_file, line);
    // End of data
    if (line.find("-FILE/COMMENT") == 0) {
      break;
    }
    // TODO: read information
  }
}

void BiasSinexFileReader::ReadInputAcknowledgments(std::ifstream& bias_sinex_file) {
  std::string line;
  while (1) {
    std::getline(bias_sinex_file, line);
    // End of data
    if (line.find("-INPUT/ACKNOWLEDGMENTS") == 0) {
      break;
    }
    // TODO: read information
  }
}

void BiasSinexFileReader::ReadBiasDescription(std::ifstream& bias_sinex_file) {
  std::string line;
  while (1) {
    std::getline(bias_sinex_file, line);
    // End of data
    if (line.find("-BIAS/DESCRIPTION") == 0) {
      break;
    }
    // TODO: read information
  }
}

void BiasSinexFileReader::ReadBiasReceiverInformation(std::ifstream& bias_sinex_file) {
  std::string line;
  while (1) {
    std::getline(bias_sinex_file, line);
    // End of data
    if (line.find("-BIAS/RECEIVER_INFORMATION") == 0) {
      break;
    }
    // TODO: read information
  }
}

void BiasSinexFileReader::ReadBiasSolution(std::ifstream& bias_sinex_file) {
  std::string line;

  while (1) {
    std::getline(bias_sinex_file, line);
    // End of data
    if (line.find("-BIAS/SOLUTION") == 0) {
      break;
    }
    // Skip comments
    if (line.find("*") == 0) {
      continue;
    }
    // data read
    BiasSolutionData solution_data;
    size_t read_point = 1;
    size_t read_size = 4;

    solution_data.SetIdentifier(line.substr(read_point, read_size));
    read_point += read_size + 1;

    read_size = 4;
    solution_data.SetSatelliteSvnCode(line.substr(read_point, read_size));
    read_point += read_size + 1;

    read_size = 3;
    solution_data.SetSatelliteNumber(line.substr(read_point, read_size));
    read_point += read_size + 1;

    read_size = 9;
    solution_data.SetStationName(line.substr(read_point, read_size));
    read_point += read_size + 1;

    read_size = 4;
    std::string obs1 = line.substr(read_point, read_size);
    read_point += read_size + 1;
    read_size = 4;
    std::string obs2 = line.substr(read_point, read_size);
    read_point += read_size + 1;
    solution_data.SetTargetSignal(obs1, obs2);

    read_size = 14;
    std::string start_time = line.substr(read_point, read_size);
    read_point += read_size + 1;

    read_size = 14;
    std::string end_time = line.substr(read_point, read_size);
    read_point += read_size + 1;

    read_size = 4;
    solution_data.SetUnit(line.substr(read_point, read_size));
    read_point += read_size + 1;

    read_size = 21;
    solution_data.SetBias(std::stod(line.substr(read_point, read_size)));
    read_point += read_size + 1;

    read_size = 12;
    solution_data.SetBiasStandardDeviation(std::stod(line.substr(read_point, read_size)));
    read_point += read_size + 1;

    if (read_point + 21 < line.size()) {
      read_size = 21;
      solution_data.SetSlope_ns_s(std::stod(line.substr(read_point, read_size)));
      read_point += read_size + 1;
    }
    if (read_point + 12 < line.size()) {
      read_size = 12;
      solution_data.SetSlopeStandardDeviation_ns_s(std::stod(line.substr(read_point, read_size)));
      read_point += read_size + 1;
    }

    solution_data_.push_back(solution_data);
  }
}

void BiasSolutionData::SetIdentifier(const std::string identifier) {
  if (identifier == "DSB ") {
    identifier_ = BiasIdentifier::kDsb;
  } else if (identifier == "ISB ") {
    identifier_ = BiasIdentifier::kIsb;
  } else if (identifier == "OSB ") {
    identifier_ = BiasIdentifier::kOsb;
  } else {
    identifier_ = BiasIdentifier::kError;
  }
}

void BiasSolutionData::SetUnit(const std::string unit) {
  if (unit == "ns  ") {
    unit_ = BiasUnit::kNs;
  } else if (unit == "cyc ") {
    unit_ = BiasUnit::kCycle;
  } else {
    unit_ = BiasUnit::kError;
  }
}

void BiasSolutionData::SetTargetSignal(const std::string signal1, const std::string signal2) {
  // TODO: Add other target combination
  if (signal1 == "C1W " && signal2 == "C2W ") {
    target_signal_ = BiasTargetSignal::kP1P2;
  } else if (signal1 == "C1P " && signal2 == "C2P ") {
    target_signal_ = BiasTargetSignal::kP1P2;
  } else if (signal1 == "C1P " && signal2 == "C1C ") {
    target_signal_ = BiasTargetSignal::kP1C1;
  } else if (signal1 == "C2P " && signal2 == "C2C ") {
    target_signal_ = BiasTargetSignal::kP2C2;
  } else {
    target_signal_ = BiasTargetSignal::kError;
  }
}

}  // namespace gnss
