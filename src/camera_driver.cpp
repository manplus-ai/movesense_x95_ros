// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/camera_driver.h"

#include <ros/ros.h>

namespace movesense_x95_ros {

CameraDriver::CameraDriver() {}

CameraDriver::~CameraDriver() {}

bool CameraDriver::Init(const CameraConfig& cfg)
{
    return CameraSession::Init(cfg, [](movesense_x95::LogLevel level, const std::string& message) {
        if (level == movesense_x95::LogLevel::Error) {
            ROS_ERROR("%s", message.c_str());
        } else if (level == movesense_x95::LogLevel::Warning) {
            ROS_WARN("%s", message.c_str());
        } else {
            ROS_INFO("%s", message.c_str());
        }
    });
}

} // namespace movesense_x95_ros
