// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include <array>
#include <string>

namespace movesense_x95_ros {

constexpr int kRoiStreamCount = 6;

inline const char* RoiStreamName(int stream)
{
    static const char* const names[kRoiStreamCount] = { "left_raw", "right_raw", "color_raw", "left_rect", "right_rect", "color_rect" };

    if (stream < 0 || stream >= kRoiStreamCount) {
        return "unknown";
    }

    return names[stream];
}

struct RoiConfig {
    bool enable = false;
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
};

struct CameraConfig {
    std::string cameraIp = "192.168.1.70";
    bool alignTimeOnStart = false;

    int fps = 25;
    int stereoWidth = 640;
    int stereoHeight = 480;
    int downsampleMode = 0;
    int depthWidth = 640;
    int depthHeight = 480;
    int colorWidth = 640;
    int colorHeight = 480;
    bool enableStereo = false;
    bool enableDepth = true;
    bool enableColor = true;
    bool enableImu = true;
    bool enableDetection = false;
    bool stereoAutoExpo = false;
    int stereoExposureUs = 5000;
    int stereoGainX = 1;
    int stereoMaxExposureUs = 65535;
    int stereoMinExposureUs = 100;
    int stereoMaxGainX = 16;
    int stereoMinGainX = 1;
    bool colorAutoExpo = false;
    int colorExposureUs = 5000;
    int colorGainX = 1;
    int colorMaxExposureUs = 65535;
    int colorMinExposureUs = 100;
    int colorMaxGainX = 16;
    int colorMinGainX = 1;
    int doePower = 255;
    int registration = 1;
    std::array<RoiConfig, kRoiStreamCount> roi;
    int frameTimeoutMs = 500;
    int firstFrameWaitMs = 5000;
    int imuPollMs = 2;
    int frameQueueSize = 8;
    int imuQueueSize = 400;
    std::string imuCalibDir = "";
    std::string cam0Topic = "/movesense/right/image_rect_raw";
    std::string cam1Topic = "/movesense/left/image_rect_raw";
    std::string colorTopic = "/movesense/color/image_rect_color";
    std::string colorInfoTopic = "/movesense/color/camera_info";
    std::string depthTopic = "/movesense/depth/image_rect_raw";
    std::string depthInfoTopic = "/movesense/depth/camera_info";
    std::string imuTopic = "/movesense/imu";
    std::string detTopic = "/movesense/detections";
    bool publishTf = true;

    std::string baseFrameId = "movesense_link";
    std::string cam0FrameId = "movesense_infra2_optical_frame";
    std::string cam1FrameId = "movesense_infra1_optical_frame";
    std::string colorFrameId = "movesense_color_optical_frame";
    std::string imuFrameId = "movesense_imu_frame";
    std::string detFrameId = "movesense_color_optical_frame";
};

} // namespace movesense_x95_ros
