// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95/camera_session.h"
#include "movesense_x95/thread_safe_queue.h"

#include <atomic>
#include <thread>

namespace movesense {
class MovesenseFrame;
struct Imu;
}

namespace movesense_x95 {

bool CopyFrame(const movesense::MovesenseFrame& source, StereoFrame& target, uint64_t minimumTimestampUs = 0);
bool ConvertImuSample(const movesense::Imu& source, ImuSample& target, uint64_t minimumTimestampUs = 0);

class FrameReceiver {
public:
    FrameReceiver();
    ~FrameReceiver();

    bool Init(movesense::Simou3Camera* cam, StereoFrameQueue* queue, int timeoutMs, LogSink log, uint64_t minimumTimestampUs = 0);
    void Shutdown();
    bool Failed() const;

private:
    void Loop();

    movesense::Simou3Camera* m_cam = nullptr;
    StereoFrameQueue* m_queue = nullptr;
    int m_timeoutMs = 500;
    uint64_t m_minimumTimestampUs = 0;
    LogSink m_log;
    std::atomic<bool> m_running { false };
    std::atomic<bool> m_failed { false };
    std::thread m_thread;
};

class RawImuReceiver {
public:
    RawImuReceiver();
    ~RawImuReceiver();

    bool Init(movesense::Simou3Camera* cam, ImuQueue* queue, int pollMs, LogSink log, uint64_t minimumTimestampUs = 0);
    void Shutdown();
    bool Failed() const;

private:
    void Loop();

    movesense::Simou3Camera* m_cam = nullptr;
    ImuQueue* m_queue = nullptr;
    int m_pollMs = 2;
    uint64_t m_minimumTimestampUs = 0;
    LogSink m_log;
    std::atomic<bool> m_running { false };
    std::atomic<bool> m_failed { false };
    std::thread m_thread;
};

}
