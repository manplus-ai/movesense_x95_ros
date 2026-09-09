// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include <cstdint>
#include <vector>

namespace movesense_x95_ros {

struct PlaneData {
    bool valid = false;
    uint32_t width = 0;
    uint32_t height = 0;
    uint8_t format = 0;
    uint64_t ptsUs = 0;
    std::vector<uint8_t> data;
};

struct StereoFrame {
    uint32_t frameCnt = 0;
    PlaneData left;
    PlaneData right;
    PlaneData color;
    PlaneData depth;
    PlaneData seg;
};

struct ImuSample {
    uint64_t tsUs = 0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;
};

constexpr double ACCEL_LSB_TO_G = 1.0 / 2048.0;
constexpr double GYRO_LSB_TO_DPS = 1.0 / 16.4;
constexpr double GRAVITY = 9.80665;
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

} // namespace movesense_x95_ros
