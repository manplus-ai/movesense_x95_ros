// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95_ros/common.h"
#include "movesense_x95_ros/thread_safe_queue.h"

#include <atomic>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <thread>

namespace movesense_x95_ros {

class StereoPublisher {
public:
    StereoPublisher();
    ~StereoPublisher();

    bool Init(ros::NodeHandle& nh, StereoFrameQueue* frameQueue, const std::string& cam0Topic, const std::string& cam1Topic,
        const std::string& colorTopic, const std::string& depthTopic, const std::string& detTopic, const std::string& cam0FrameId,
        const std::string& cam1FrameId, const std::string& colorFrameId, const std::string& depthFrameId, const std::string& detFrameId,
        int queueSize, bool enableStereo, bool enableColor, bool enableDetection);
    void Shutdown();

    void SetDepthCameraInfo(ros::NodeHandle& nh, const std::string& topic, const sensor_msgs::CameraInfo& info, int queueSize);
    void SetColorCameraInfo(ros::NodeHandle& nh, const std::string& topic, const sensor_msgs::CameraInfo& info, int queueSize);

private:
    void Loop();
    void PublishOne(ros::Publisher& pub, const PlaneData& img, const std::string& frameId);
    void PublishColor(const PlaneData& color);
    void PublishDepth(const PlaneData& depth);
    void PublishDetections(const PlaneData& seg);

    StereoFrameQueue* m_frameQueue = nullptr;
    ros::Publisher m_cam0Pub;
    ros::Publisher m_cam1Pub;
    ros::Publisher m_colorPub;
    ros::Publisher m_colorInfoPub;
    ros::Publisher m_depthPub;
    ros::Publisher m_depthInfoPub;
    ros::Publisher m_detPub;
    sensor_msgs::CameraInfo m_depthInfo;
    sensor_msgs::CameraInfo m_colorInfo;
    bool m_hasDepthInfo = false;
    bool m_hasColorInfo = false;
    bool m_stereoEnabled = false;
    bool m_colorEnabled = false;
    bool m_detEnabled = true;
    std::string m_cam0FrameId = "movesense_infra2_optical_frame";
    std::string m_cam1FrameId = "movesense_infra1_optical_frame";
    std::string m_colorFrameId = "movesense_color_optical_frame";
    std::string m_depthFrameId = "movesense_infra2_optical_frame";
    std::string m_detFrameId = "movesense_color_optical_frame";
    std::atomic<bool> m_running { false };
    std::thread m_thread;
};

} // namespace movesense_x95_ros
