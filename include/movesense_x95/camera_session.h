// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95/camera_config.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace movesense {
class Simou3Camera;
}

namespace movesense_x95 {

enum class LogLevel { Info, Warning, Error };
using LogSink = std::function<void(LogLevel, const std::string&)>;

class CameraConnectionProbe {
public:
    using ProbeFunction = std::function<bool()>;

    CameraConnectionProbe();
    ~CameraConnectionProbe();

    bool Init(const std::string& ip, LogSink log, ProbeFunction probe = ProbeFunction(), int intervalMs = 2000, bool enabled = true);
    void Shutdown();
    bool Failed() const;

private:
    void Loop();

    std::string m_ip;
    LogSink m_log;
    ProbeFunction m_probe;
    int m_intervalMs = 2000;
    std::atomic<bool> m_probeRunning { false };
    std::atomic<bool> m_connectionFailed { false };
    std::mutex m_probeMutex;
    std::condition_variable m_probeWake;
    std::thread m_probeThread;
};

class CameraSession {
public:
    CameraSession();
    ~CameraSession();

    bool Init(const CameraConfig& cfg, LogSink log);
    bool ValidateConfiguration(const CameraConfig& cfg);
    void Shutdown(bool connectionLost = false);

    movesense::Simou3Camera* Camera();
    bool IsPassive() const;
    int CameraType() const;
    double DepthUnitMm() const;
    int TriggerMode() const;
    uint64_t MinimumTimestampUs() const;
    bool Failed() const;

private:
    bool CheckCommand(const char* name, int result);
    bool ValidateConfig(const CameraConfig& cfg);
    bool ApplyRoi(const CameraConfig& cfg);
    bool ApplyExposure(const CameraConfig& cfg, bool color);
    bool WaitFirstFrame(int totalMs);
    bool WaitFirstImu(int totalMs);
    void Log(LogLevel level, const std::string& message) const;

    movesense::Simou3Camera* m_cam = nullptr;
    LogSink m_log;
    bool m_settingsOpened = false;
    bool m_streaming = false;
    bool m_isPassive = false;
    int m_cameraType = -1;
    bool m_roiActive = false;
    bool m_dataClosed = false;
    double m_depthUnitMm = 0.0;
    int m_triggerMode = -1;
    uint64_t m_minimumTimestampUs = 0;
    CameraConnectionProbe m_connectionProbe;
};

}
