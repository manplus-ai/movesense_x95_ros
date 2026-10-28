// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95/calibration_data.h"

namespace movesense {
class Simou3Camera;
}

namespace movesense_x95_ros {

class StereoCalib : public movesense_x95::CalibrationData {
public:
    StereoCalib();
    ~StereoCalib();

    bool Init(movesense::Simou3Camera* cam);
    bool InitColor(movesense::Simou3Camera* cam);
    void LogSummary() const;
};

} // namespace movesense_x95_ros
