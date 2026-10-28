// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95/common.h"
#include "movesense_x95/thread_safe_queue.h"

#include <atomic>
#include <ros/ros.h>
#include <string>
#include <thread>

namespace movesense_x95_ros {

using movesense_x95::ImuQueue;
using movesense_x95::ImuSample;

class ImuPublisher {
public:
    ImuPublisher();
    ~ImuPublisher();

    bool Init(ros::NodeHandle& nh, ImuQueue* imuQueue, const std::string& imuTopic, const std::string& frameId, int queueSize);
    void Shutdown();

private:
    void Loop();

    ImuQueue* m_imuQueue = nullptr;
    ros::Publisher m_imuPub;
    std::string m_frameId = "movesense_imu_frame";
    std::atomic<bool> m_running { false };
    std::thread m_thread;
};

} // namespace movesense_x95_ros
