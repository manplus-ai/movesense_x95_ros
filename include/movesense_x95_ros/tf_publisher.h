// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95_ros/camera_config.h"
#include "movesense_x95_ros/stereo_calib.h"

#include <memory>
#include <tf2_ros/static_transform_broadcaster.h>

namespace movesense_x95_ros {

class TfPublisher {
public:
    TfPublisher();
    ~TfPublisher();

    bool Publish(const CameraConfig& cfg, const StereoCalib& calib, bool colorFrameUsed, bool isPassive, int cameraType);

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_broadcaster;
};

} // namespace movesense_x95_ros
