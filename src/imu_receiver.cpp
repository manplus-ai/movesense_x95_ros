// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/imu_receiver.h"

#include <chrono>
#include <movesense/Simou3Camera.h>
#include <ros/ros.h>
#include <vector>

using namespace movesense;

namespace movesense_x95_ros {

ImuReceiver::ImuReceiver() {}

ImuReceiver::~ImuReceiver() {}

bool ImuReceiver::Init(Simou3Camera* cam, ImuQueue* imuQueue, int pollMs)
{
    if (cam == nullptr || imuQueue == nullptr) {
        ROS_ERROR("[ImuReceiver] Init null argument");
        return false;
    }

    m_cam = cam;
    m_imuQueue = imuQueue;
    m_pollMs = pollMs;

    m_running.store(true);
    m_thread = std::thread(&ImuReceiver::Loop, this);

    return true;
}

void ImuReceiver::Shutdown()
{
    m_running.store(false);

    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void ImuReceiver::Loop()
{
    auto winStart = std::chrono::steady_clock::now();
    int sampleCount = 0;

    while (m_running.load()) {
        std::vector<Imu> raws;
        int n = m_cam->getIMU(raws);

        if (n > 0) {
            for (const Imu& one : raws) {
                ImuSample sample;
                sample.tsUs = one.timestampUs;
                sample.ax = static_cast<double>(one.ax) * ACCEL_LSB_TO_G * GRAVITY;
                sample.ay = static_cast<double>(one.ay) * ACCEL_LSB_TO_G * GRAVITY;
                sample.az = static_cast<double>(one.az) * ACCEL_LSB_TO_G * GRAVITY;
                sample.gx = static_cast<double>(one.gx) * GYRO_LSB_TO_DPS * DEG_TO_RAD;
                sample.gy = static_cast<double>(one.gy) * GYRO_LSB_TO_DPS * DEG_TO_RAD;
                sample.gz = static_cast<double>(one.gz) * GYRO_LSB_TO_DPS * DEG_TO_RAD;

                m_imuQueue->Push(std::move(sample));
                ++sampleCount;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(m_pollMs));
        }

        auto now = std::chrono::steady_clock::now();
        double sec = std::chrono::duration<double>(now - winStart).count();

        if (sec >= 5.0) {
            ROS_INFO("[rate] IMU %.1f Hz (%d samples / %.1fs)", sampleCount / sec, sampleCount, sec);

            winStart = now;
            sampleCount = 0;
        }
    }
}

} // namespace movesense_x95_ros
