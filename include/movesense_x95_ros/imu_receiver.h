// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95_ros/common.h"
#include "movesense_x95_ros/thread_safe_queue.h"

#include <atomic>
#include <thread>

namespace movesense {
class Simou3Camera;
}
using movesense::Simou3Camera;

namespace movesense_x95_ros {

class ImuReceiver {
public:
    ImuReceiver();
    ~ImuReceiver();

    bool Init(Simou3Camera* cam, ImuQueue* imuQueue, int pollMs);
    void Shutdown();

private:
    void Loop();

    Simou3Camera* m_cam = nullptr;
    ImuQueue* m_imuQueue = nullptr;
    int m_pollMs = 2;
    std::atomic<bool> m_running { false };
    std::thread m_thread;
};

} // namespace movesense_x95_ros
