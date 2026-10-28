// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95/calibration_data.h"

#include <string>

namespace movesense_x95 {

bool WriteCalibrationXml(const CalibrationData& calib, const std::string& directory, int cameraType, std::string& error);

}
