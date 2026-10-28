// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95/camera_session.h"

namespace movesense_x95_ros {

using movesense_x95::CameraConfig;

class CameraDriver : public movesense_x95::CameraSession {
public:
    CameraDriver();
    ~CameraDriver();

    bool Init(const CameraConfig& cfg);
};

} // namespace movesense_x95_ros
