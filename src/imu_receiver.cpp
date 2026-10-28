// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/imu_receiver.h"

#include <ros/ros.h>

namespace movesense_x95_ros {

ImuReceiver::ImuReceiver() {}

ImuReceiver::~ImuReceiver() {}

bool ImuReceiver::Init(movesense::Simou3Camera* cam, ImuQueue* queue, int pollMs, uint64_t minimumTimestampUs)
{
    return RawImuReceiver::Init(
        cam, queue, pollMs,
        [](movesense_x95::LogLevel level, const std::string& message) {
            if (level == movesense_x95::LogLevel::Error) {
                ROS_ERROR("%s", message.c_str());
            } else if (level == movesense_x95::LogLevel::Warning) {
                ROS_WARN("%s", message.c_str());
            } else {
                ROS_INFO("%s", message.c_str());
            }
        },
        minimumTimestampUs);
}

} // namespace movesense_x95_ros
