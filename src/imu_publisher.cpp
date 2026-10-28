// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/imu_publisher.h"

#include <exception>
#include <sensor_msgs/Imu.h>

namespace movesense_x95_ros {

ImuPublisher::ImuPublisher() {}

ImuPublisher::~ImuPublisher() {}

bool ImuPublisher::Init(ros::NodeHandle& nh, ImuQueue* imuQueue, const std::string& imuTopic, const std::string& frameId, int queueSize)
{
    if (imuQueue == nullptr || queueSize < 1 || imuTopic.empty() || frameId.empty() || m_thread.joinable()) {
        ROS_ERROR("[ImuPublisher] invalid initialization");
        return false;
    }

    m_imuQueue = imuQueue;
    m_frameId = frameId;

    try {
        m_imuPub = nh.advertise<sensor_msgs::Imu>(imuTopic, queueSize);
        m_running.store(true);
        m_thread = std::thread(&ImuPublisher::Loop, this);
    } catch (const std::exception& error) {
        m_running.store(false);
        m_imuQueue = nullptr;
        m_imuPub.shutdown();
        ROS_ERROR("[ImuPublisher] initialization failed: %s", error.what());
        return false;
    }

    return true;
}

void ImuPublisher::Shutdown()
{
    m_running.store(false);
    if (m_imuQueue != nullptr) {
        m_imuQueue->Stop();
    }

    if (m_thread.joinable()) {
        m_thread.join();
    }
    m_imuQueue = nullptr;
    m_imuPub.shutdown();
}

void ImuPublisher::Loop()
{
    while (m_running.load()) {
        ImuSample sample;

        if (!m_imuQueue->Pop(sample)) {
            break;
        }

        sensor_msgs::Imu msg;

        if (sample.tsUs == 0) {
            ROS_ERROR_THROTTLE(1.0, "[ImuPublisher] IMU sample has invalid timestamp (tsUs==0)");
            continue;
        }
        msg.header.stamp.fromNSec(sample.tsUs * 1000ull);

        msg.header.frame_id = m_frameId;

        msg.linear_acceleration.x = sample.ax;
        msg.linear_acceleration.y = sample.ay;
        msg.linear_acceleration.z = sample.az;

        msg.angular_velocity.x = sample.gx;
        msg.angular_velocity.y = sample.gy;
        msg.angular_velocity.z = sample.gz;

        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        msg.orientation_covariance[0] = -1.0;

        m_imuPub.publish(msg);
    }
}

} // namespace movesense_x95_ros
