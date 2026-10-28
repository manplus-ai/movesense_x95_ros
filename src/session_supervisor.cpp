// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95/session_supervisor.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <limits>
#include <thread>
#include <utility>

namespace movesense_x95 {

SessionSupervisor::SessionSupervisor() { }

SessionSupervisor::~SessionSupervisor() { }

bool SessionSupervisor::Init(AttemptFunction attempt, RunningFunction running, SpinFunction spin, LogSink log, int retryMs)
{
    if (!attempt || !running || !spin || retryMs < 1) {
        WriteLog(log, LogLevel::Error, "Invalid session supervisor initialization");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
        if (!m_runActive.load()) {
            m_attempt = std::move(attempt);
            m_running = std::move(running);
            m_spin = std::move(spin);
            m_log = std::move(log);
            m_retryMs = retryMs;
            m_initialized = true;
            m_hasRun = false;
            m_stopRequested.store(false);
            return true;
        }
    }

    WriteLog(log, LogLevel::Error, "Cannot initialize an active session supervisor");
    return false;
}

int SessionSupervisor::Run()
{
    bool started = false;
    LogSink log;
    {
        std::lock_guard<std::mutex> lock(m_lifecycleMutex);
        log = m_log;
        if (m_initialized && !m_hasRun && !m_runActive.load()) {
            m_runActive.store(true);
            m_hasRun = true;
            started = true;
        }
    }
    if (!started) {
        WriteLog(log, LogLevel::Error, "Session supervisor is uninitialized or has already run");
        return 1;
    }

    int result = 1;
    try {
        result = Loop();
    } catch (const std::exception& error) {
        WriteLog(log, LogLevel::Error, std::string("Session supervisor callback failed: ") + error.what());
    } catch (...) {
        WriteLog(log, LogLevel::Error, "Session supervisor callback failed with an unknown exception");
    }
    m_runActive.store(false);
    return result;
}

void SessionSupervisor::Shutdown()
{
    m_stopRequested.store(true);
}

void SessionSupervisor::WriteLog(const LogSink& log, LogLevel level, const std::string& message)
{
    if (!log) {
        return;
    }
    try {
        log(level, message);
    } catch (...) {
    }
}

bool SessionSupervisor::Continue() const
{
    return !m_stopRequested.load() && m_running();
}

int SessionSupervisor::Loop()
{
    uint64_t attemptId = 0;
    while (Continue()) {
        if (attemptId == std::numeric_limits<uint64_t>::max()) {
            WriteLog(m_log, LogLevel::Error, "Session attempt counter exhausted");
            return 1;
        }

        const SessionResult result = m_attempt(++attemptId);
        if (result == SessionResult::Stopped || !Continue()) {
            return 0;
        }
        if (result != SessionResult::Retry) {
            WriteLog(m_log, LogLevel::Error, "Camera session stopped with a fatal error");
            return 1;
        }

        WriteLog(m_log, LogLevel::Warning, "Camera session unavailable; retrying in " + std::to_string(m_retryMs) + " ms");
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_retryMs);
        while (Continue()) {
            m_spin();
            if (!Continue()) {
                return 0;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                break;
            }
            std::this_thread::sleep_for(std::min(deadline - now,
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(20))));
        }
    }

    return 0;
}

}
