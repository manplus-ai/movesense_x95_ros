// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95_ros/camera_config.h"

namespace movesense {
class Simou3Camera;
}
using movesense::Simou3Camera;

namespace movesense_x95_ros {

class CameraDriver {
public:
    CameraDriver();
    ~CameraDriver();

    bool Init(const CameraConfig& cfg);
    void Shutdown();

    Simou3Camera* Camera()
    {
        return m_cam;
    }

    bool IsPassive() const
    {
        return m_isPassive;
    }

    int CameraType() const
    {
        return m_cameraType;
    }

private:
    bool WaitFirstFrame(int totalMs);

    Simou3Camera* m_cam = nullptr;
    bool m_settingsOpened = false;
    bool m_streaming = false;
    bool m_isPassive = false;
    int m_cameraType = 0;
};

} // namespace movesense_x95_ros
