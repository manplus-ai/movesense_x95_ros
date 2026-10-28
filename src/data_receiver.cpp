// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95/data_receiver.h"

#include <chrono>
#include <exception>
#include <movesense/Simou3Camera.h>
#include <utility>
#include <vector>

namespace movesense_x95 {

static void CopyPlane(const movesense::MovesenseFrame::Plane& source, PlaneData& target, bool image, uint64_t minimumTimestampUs)
{
    if (!source.has || source.status != 0 || !source.complete() || source.data.empty() || source.pts == 0 || source.pts < minimumTimestampUs ||
        (image && (source.width == 0 || source.height == 0))) {
        return;
    }
    target.width = source.width;
    target.height = source.height;
    target.format = static_cast<uint8_t>(source.format);
    target.ptsUs = source.pts;
    target.data = source.data;
    target.valid = true;
}

bool CopyFrame(const movesense::MovesenseFrame& source, StereoFrame& target, uint64_t minimumTimestampUs)
{
    target = StereoFrame();
    target.frameCnt = source.frameCnt();
    CopyPlane(source.leftRect(), target.left, true, minimumTimestampUs);
    CopyPlane(source.rightRect(), target.right, true, minimumTimestampUs);
    CopyPlane(source.rgbRect(), target.color, true, minimumTimestampUs);
    CopyPlane(source.depth(), target.depth, true, minimumTimestampUs);
    CopyPlane(source.seg(), target.seg, false, minimumTimestampUs);
    return target.left.valid || target.right.valid || target.color.valid || target.depth.valid || target.seg.valid;
}

bool ConvertImuSample(const movesense::Imu& source, ImuSample& target, uint64_t minimumTimestampUs)
{
    if (source.timestampUs == 0 || source.timestampUs < minimumTimestampUs) {
        return false;
    }
    target.tsUs = source.timestampUs;
    target.ax = static_cast<double>(source.ax) * ACCEL_LSB_TO_G * GRAVITY;
    target.ay = static_cast<double>(source.ay) * ACCEL_LSB_TO_G * GRAVITY;
    target.az = static_cast<double>(source.az) * ACCEL_LSB_TO_G * GRAVITY;
    target.gx = static_cast<double>(source.gx) * GYRO_LSB_TO_DPS * DEG_TO_RAD;
    target.gy = static_cast<double>(source.gy) * GYRO_LSB_TO_DPS * DEG_TO_RAD;
    target.gz = static_cast<double>(source.gz) * GYRO_LSB_TO_DPS * DEG_TO_RAD;
    return true;
}

FrameReceiver::FrameReceiver() {}

FrameReceiver::~FrameReceiver() {}

bool FrameReceiver::Init(movesense::Simou3Camera* cam, StereoFrameQueue* queue, int timeoutMs, LogSink log, uint64_t minimumTimestampUs)
{
    if (cam == nullptr || queue == nullptr || timeoutMs < 10 || timeoutMs > 5000 || m_thread.joinable()) {
        if (log) {
            log(LogLevel::Error, "invalid frame receiver initialization");
        }
        return false;
    }
    m_cam = cam;
    m_queue = queue;
    m_timeoutMs = timeoutMs;
    m_minimumTimestampUs = minimumTimestampUs;
    m_log = std::move(log);
    m_failed.store(false);
    m_running.store(true);
    try {
        m_thread = std::thread(&FrameReceiver::Loop, this);
    } catch (const std::exception& error) {
        m_running.store(false);
        if (m_log) {
            m_log(LogLevel::Error, std::string("frame receiver thread failed: ") + error.what());
        }
        return false;
    }
    return true;
}

void FrameReceiver::Shutdown()
{
    m_running.store(false);
    if (m_thread.joinable()) {
        m_thread.join();
    }
    m_cam = nullptr;
    m_queue = nullptr;
}

void FrameReceiver::Loop()
{
    auto windowStart = std::chrono::steady_clock::now();
    int received = 0;
    while (m_running.load()) {
        movesense::MovesenseFrame frame;
        if (m_cam->getFrame(frame, m_timeoutMs)) {
            StereoFrame payload;
            if (CopyFrame(frame, payload, m_minimumTimestampUs)) {
                m_queue->Push(std::move(payload));
                ++received;
            }
        } else {
            std::vector<movesense::Imu> probe;
            if (m_running.load() && m_cam->getIMU(probe, 0) < 0) {
                m_failed.store(true);
                m_running.store(false);
                if (m_log) {
                    m_log(LogLevel::Error, "data connection closed; frame receiver stopped");
                }
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        const auto now = std::chrono::steady_clock::now();
        const double seconds = std::chrono::duration<double>(now - windowStart).count();
        if (seconds >= 5.0) {
            if (m_log) {
                m_log(LogLevel::Info, "image rate=" + std::to_string(received / seconds) + " fps");
            }
            windowStart = now;
            received = 0;
        }
    }
}

RawImuReceiver::RawImuReceiver() {}

RawImuReceiver::~RawImuReceiver() {}

bool RawImuReceiver::Init(movesense::Simou3Camera* cam, ImuQueue* queue, int pollMs, LogSink log, uint64_t minimumTimestampUs)
{
    if (cam == nullptr || queue == nullptr || pollMs < 1 || pollMs > 1000 || m_thread.joinable()) {
        if (log) {
            log(LogLevel::Error, "invalid IMU receiver initialization");
        }
        return false;
    }
    m_cam = cam;
    m_queue = queue;
    m_pollMs = pollMs;
    m_minimumTimestampUs = minimumTimestampUs;
    m_log = std::move(log);
    m_failed.store(false);
    m_running.store(true);
    try {
        m_thread = std::thread(&RawImuReceiver::Loop, this);
    } catch (const std::exception& error) {
        m_running.store(false);
        if (m_log) {
            m_log(LogLevel::Error, std::string("IMU receiver thread failed: ") + error.what());
        }
        return false;
    }
    return true;
}

void RawImuReceiver::Shutdown()
{
    m_running.store(false);
    if (m_thread.joinable()) {
        m_thread.join();
    }
    m_cam = nullptr;
    m_queue = nullptr;
}

void RawImuReceiver::Loop()
{
    auto windowStart = std::chrono::steady_clock::now();
    int received = 0;
    while (m_running.load()) {
        std::vector<movesense::Imu> samples;
        const int count = m_cam->getIMU(samples);
        if (count < 0) {
            if (m_running.load()) {
                m_failed.store(true);
                if (m_log) {
                    m_log(LogLevel::Error, "data connection closed; IMU receiver stopped");
                }
            }
            m_running.store(false);
            break;
        }
        for (const movesense::Imu& raw : samples) {
            ImuSample sample;
            if (!ConvertImuSample(raw, sample, m_minimumTimestampUs)) {
                continue;
            }
            m_queue->Push(std::move(sample));
            ++received;
        }
        if (count == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(m_pollMs));
        }
        const auto now = std::chrono::steady_clock::now();
        const double seconds = std::chrono::duration<double>(now - windowStart).count();
        if (seconds >= 5.0) {
            if (m_log) {
                m_log(LogLevel::Info, "IMU rate=" + std::to_string(received / seconds) + " Hz");
            }
            windowStart = now;
            received = 0;
        }
    }
}

bool FrameReceiver::Failed() const
{
    return m_failed.load();
}

bool RawImuReceiver::Failed() const
{
    return m_failed.load();
}

} // namespace movesense_x95
