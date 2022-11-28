#pragma once

#include <Component/Abstract/SensorBase.h>

// TODO:これをコアに移して全体で共有したい
template <size_t N>
SensorBase<N> ReadSensorBaseInformation(const std::string file_name, const double step_width_s);

#include "InitializeSensorBase_tfs.hpp"
