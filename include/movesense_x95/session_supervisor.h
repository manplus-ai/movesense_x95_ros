// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95/camera_session.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

namespace movesense_x95 {

enum class SessionResult { Stopped, Retry, Fatal };

class SessionSupervisor {
public:
    using AttemptFunction = std::function<SessionResult(uint64_t)>;
    using RunningFunction = std::function<bool()>;
    using SpinFunction = std::function<void()>;

    SessionSupervisor();
    ~SessionSupervisor();

    bool Init(AttemptFunction attempt, RunningFunction running, SpinFunction spin, LogSink log, int retryMs = 1000);
    int Run();
    void Shutdown();

private:
    static void WriteLog(const LogSink& log, LogLevel level, const std::string& message);
    bool Continue() const;
    int Loop();

    AttemptFunction m_attempt;
    RunningFunction m_running;
    SpinFunction m_spin;
    LogSink m_log;
    int m_retryMs = 1000;
    bool m_initialized = false;
    bool m_hasRun = false;
    std::atomic<bool> m_runActive { false };
    std::atomic<bool> m_stopRequested { false };
    std::mutex m_lifecycleMutex;
};

}
