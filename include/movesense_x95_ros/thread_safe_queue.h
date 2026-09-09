// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95_ros/common.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <utility>

namespace movesense_x95_ros {

template<typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t capacity = 8) : m_capacity(capacity), m_stopped(false) {}

    void Push(T&& item)
    {
        std::lock_guard<std::mutex> lock(m_mtx);

        if (m_stopped) {
            return;
        }

        while (m_queue.size() >= m_capacity) {
            m_queue.pop_front();
            ++m_dropped;
        }

        m_queue.push_back(std::move(item));
        m_cv.notify_one();
    }

    bool Pop(T& out)
    {
        std::unique_lock<std::mutex> lock(m_mtx);

        m_cv.wait(lock, [this]() { return m_stopped || !m_queue.empty(); });

        if (m_queue.empty()) {
            return false;
        }

        out = std::move(m_queue.front());
        m_queue.pop_front();

        return true;
    }

    void Stop()
    {
        std::lock_guard<std::mutex> lock(m_mtx);

        m_stopped = true;
        m_cv.notify_all();
    }

    uint64_t Dropped() const
    {
        std::lock_guard<std::mutex> lock(m_mtx);

        return m_dropped;
    }

private:
    mutable std::mutex m_mtx;
    std::condition_variable m_cv;
    std::deque<T> m_queue;
    size_t m_capacity;
    bool m_stopped;
    uint64_t m_dropped = 0;
};

using StereoFrameQueue = ThreadSafeQueue<StereoFrame>;
using ImuQueue = ThreadSafeQueue<ImuSample>;

} // namespace movesense_x95_ros
