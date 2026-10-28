// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95/camera_session.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <exception>
#include <movesense/Simou3Camera.h>
#include <movesense/Simou3CameraScan.h>
#include <movesense/Simou3Types.h>
#include <movesense/transfer_mode_def.h>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace movesense_x95 {

using movesense::Simou3Camera;

CameraConnectionProbe::CameraConnectionProbe() {}

CameraConnectionProbe::~CameraConnectionProbe() {}

bool CameraConnectionProbe::Init(const std::string& ip, LogSink log, ProbeFunction probe, int intervalMs, bool enabled)
{
    if (ip.empty() || intervalMs < 1 || m_probeThread.joinable()) {
        if (log) {
            log(LogLevel::Error, "invalid connection probe initialization");
        }
        return false;
    }
    m_ip = ip;
    m_log = std::move(log);
    m_probe = std::move(probe);
    m_intervalMs = intervalMs;
    m_connectionFailed.store(false);
    if (!enabled) {
        return true;
    }
    m_probeRunning.store(true);
    try {
        m_probeThread = std::thread(&CameraConnectionProbe::Loop, this);
    } catch (const std::exception& error) {
        m_probeRunning.store(false);
        if (m_log) {
            m_log(LogLevel::Error, std::string("connection probe thread failed: ") + error.what());
        }
        return false;
    }
    return true;
}

void CameraConnectionProbe::Shutdown()
{
    {
        std::lock_guard<std::mutex> lock(m_probeMutex);
        m_probeRunning.store(false);
    }
    m_probeWake.notify_all();
    if (m_probeThread.joinable()) {
        m_probeThread.join();
    }
}

void CameraConnectionProbe::Loop()
{
    movesense::Simou3CameraScan scanner(5004);
    unsigned failures = 0;
    while (m_probeRunning.load()) {
        const auto nextProbe = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_intervalMs);
        bool online = false;
        try {
            movesense::CameraInfo information;
            online = m_probe ? m_probe() : scanner.scanCamera(m_ip, information) == 0;
        } catch (const std::exception& error) {
            if (m_log) {
                m_log(LogLevel::Warning, std::string("connection probe failed: ") + error.what());
            }
        } catch (...) {
            if (m_log) {
                m_log(LogLevel::Warning, "connection probe failed with an unknown exception");
            }
        }
        if (!m_probeRunning.load()) {
            break;
        }
        if (online) {
            failures = 0;
        } else {
            ++failures;
            if (failures >= 3) {
                m_connectionFailed.store(true);
                m_probeRunning.store(false);
                if (m_log) {
                    m_log(LogLevel::Error, "UDP discovery failed three consecutive times; camera connection is unavailable");
                }
                break;
            }
        }
        std::unique_lock<std::mutex> lock(m_probeMutex);
        m_probeWake.wait_until(lock, nextProbe, [this]() { return !m_probeRunning.load(); });
    }
}

static int ResToDown(int width, int height)
{
    if (width == 1280 && height == 960) {
        return 0;
    }
    if (width == 640 && height == 480) {
        return 1;
    }
    return -1;
}

static bool ValidExposure(bool automatic, int exposure, int gain, int minimumExposure, int maximumExposure, int minimumGain, int maximumGain)
{
    const auto optional = [](int value, int maximum) { return value == -1 || (value >= 1 && value <= maximum); };

    if (!optional(minimumExposure, 65535) || !optional(maximumExposure, 65535) || !optional(minimumGain, 16) || !optional(maximumGain, 16)) {
        return false;
    }
    if ((minimumExposure > 0 && maximumExposure > 0 && minimumExposure > maximumExposure) ||
        (minimumGain > 0 && maximumGain > 0 && minimumGain > maximumGain)) {
        return false;
    }
    if (!automatic && (exposure < 1 || exposure > 65535 || gain < 1 || gain > 16)) {
        return false;
    }
    if (!automatic && ((minimumExposure > 0 && exposure < minimumExposure) || (maximumExposure > 0 && exposure > maximumExposure) ||
                          (minimumGain > 0 && gain < minimumGain) || (maximumGain > 0 && gain > maximumGain))) {
        return false;
    }
    return true;
}

CameraSession::CameraSession() {}

CameraSession::~CameraSession() {}

void CameraSession::Log(LogLevel level, const std::string& message) const
{
    if (m_log) {
        m_log(level, message);
    }
}

bool CameraSession::CheckCommand(const char* name, int result)
{
    if (result > 0) {
        return true;
    }
    Log(LogLevel::Error, std::string(name) + " failed (rc=" + std::to_string(result) + ")");
    return false;
}

bool CameraSession::ValidateConfiguration(const CameraConfig& cfg)
{
    return ValidateConfig(cfg);
}

bool CameraSession::ValidateConfig(const CameraConfig& cfg)
{
    if (cfg.cameraIp.empty() || cfg.fps < 5 || cfg.fps > 25 || cfg.downsampleMode < -1 || cfg.downsampleMode > 1 || cfg.registration < -1 ||
        cfg.registration > 1 || (cfg.enableDepth && cfg.registration < 0) ||
        (cfg.triggerMode != -1 && cfg.triggerMode != movesense::kTriggerModeAuto && cfg.triggerMode != movesense::kTriggerModeExternal) ||
        cfg.doePower < -1 || cfg.doePower > 255 || cfg.frameTimeoutMs < 10 || cfg.frameTimeoutMs > 5000 || cfg.firstFrameWaitMs < 1 ||
        cfg.firstFrameWaitMs > 60000 || cfg.imuPollMs < 1 || cfg.imuPollMs > 1000 || cfg.frameQueueSize < 1 || cfg.imuQueueSize < 1) {
        Log(LogLevel::Error, "invalid camera, timing, queue, trigger, registration, FPS or DOE configuration");
        return false;
    }
    if (!cfg.enableStereo && !cfg.enableDepth && !cfg.enableColor && !cfg.enableDetection && !cfg.enableImu) {
        Log(LogLevel::Error, "at least one stream must be enabled");
        return false;
    }
    if (!ValidExposure(cfg.stereoAutoExpo, cfg.stereoExposureUs, cfg.stereoGainX, cfg.stereoMinExposureUs, cfg.stereoMaxExposureUs,
            cfg.stereoMinGainX, cfg.stereoMaxGainX) ||
        !ValidExposure(cfg.colorAutoExpo, cfg.colorExposureUs, cfg.colorGainX, cfg.colorMinExposureUs, cfg.colorMaxExposureUs, cfg.colorMinGainX,
            cfg.colorMaxGainX)) {
        Log(LogLevel::Error, "invalid exposure or gain configuration");
        return false;
    }
    const int stereoDown = ResToDown(cfg.stereoWidth, cfg.stereoHeight);
    const int depthDown = ResToDown(cfg.depthWidth, cfg.depthHeight);
    if (stereoDown < 0 || depthDown < 0 || ResToDown(cfg.colorWidth, cfg.colorHeight) < 0 || (stereoDown == 1 && depthDown == 0) ||
        (stereoDown == 0 && depthDown == 1 && cfg.downsampleMode == 0)) {
        Log(LogLevel::Error, "invalid resolution combination or downsample mode");
        return false;
    }
    for (int i = 0; i < kRoiStreamCount; ++i) {
        const RoiConfig& roi = cfg.roi[i];
        if (roi.enable && (roi.x1 < 0 || roi.x1 > 1280 || roi.y1 < 0 || roi.y1 > 960 || roi.x2 < 0 || roi.x2 > 1280 || roi.y2 < 0 || roi.y2 > 960 ||
                              roi.x2 - roi.x1 < 16 || roi.y2 - roi.y1 < 16 || ((roi.x1 | roi.y1 | roi.x2 | roi.y2) & 1) != 0)) {
            Log(LogLevel::Error, std::string("invalid ROI rectangle for ") + RoiStreamName(i));
            return false;
        }
    }
    return true;
}

bool CameraSession::ApplyRoi(const CameraConfig& cfg)
{
    static_assert(static_cast<int>(movesense::RoiStream::LeftRaw) == 0 && static_cast<int>(movesense::RoiStream::RightRaw) == 1 &&
                      static_cast<int>(movesense::RoiStream::RgbRaw) == 2 && static_cast<int>(movesense::RoiStream::LeftRect) == 3 &&
                      static_cast<int>(movesense::RoiStream::RightRect) == 4 && static_cast<int>(movesense::RoiStream::RgbRect) == 5 &&
                      kRoiStreamCount == 6,
        "RoiConfig index must follow movesense::RoiStream");

    for (int i = 0; i < kRoiStreamCount; ++i) {
        const RoiConfig& roi = cfg.roi[i];
        const int result = m_cam->setRoi(static_cast<movesense::RoiStream>(i), roi.enable, static_cast<unsigned>(roi.enable ? roi.x1 : 0),
            static_cast<unsigned>(roi.enable ? roi.y1 : 0), static_cast<unsigned>(roi.enable ? roi.x2 : 0),
            static_cast<unsigned>(roi.enable ? roi.y2 : 0));
        if (result > 0) {
            m_roiActive = m_roiActive || roi.enable;
            continue;
        }
        for (int j = 0; j < kRoiStreamCount; ++j) {
            if (cfg.roi[j].enable) {
                Log(LogLevel::Error, std::string("setRoi failed for ") + RoiStreamName(i) + " (rc=" + std::to_string(result) + ")");
                return false;
            }
        }
        Log(LogLevel::Warning, "ROI reset is unsupported or failed; no ROI requested");
        return true;
    }
    return true;
}

bool CameraSession::ApplyExposure(const CameraConfig& cfg, bool color)
{
    const int maximumExposure = color ? cfg.colorMaxExposureUs : cfg.stereoMaxExposureUs;
    const int minimumExposure = color ? cfg.colorMinExposureUs : cfg.stereoMinExposureUs;
    const int maximumGain = color ? cfg.colorMaxGainX : cfg.stereoMaxGainX;
    const int minimumGain = color ? cfg.colorMinGainX : cfg.stereoMinGainX;
    const bool automatic = color ? cfg.colorAutoExpo : cfg.stereoAutoExpo;

    if (maximumExposure >= 0 && !CheckCommand(color ? "setRGBMaxExposure" : "setStereoMaxExposure",
                                    color ? m_cam->setRGBMaxExposure(static_cast<unsigned>(maximumExposure))
                                          : m_cam->setStereoMaxExposure(static_cast<unsigned>(maximumExposure)))) {
        return false;
    }
    if (minimumExposure >= 0 && !CheckCommand(color ? "setRGBMinExposure" : "setStereoMinExposure",
                                    color ? m_cam->setRGBMinExposure(static_cast<unsigned>(minimumExposure))
                                          : m_cam->setStereoMinExposure(static_cast<unsigned>(minimumExposure)))) {
        return false;
    }
    if (maximumGain >= 0 &&
        !CheckCommand(color ? "setRGBMaxGain" : "setStereoMaxGain",
            color ? m_cam->setRGBMaxGain(static_cast<float>(maximumGain * 128)) : m_cam->setStereoMaxGain(static_cast<float>(maximumGain * 128)))) {
        return false;
    }
    if (minimumGain >= 0 &&
        !CheckCommand(color ? "setRGBMinGain" : "setStereoMinGain",
            color ? m_cam->setRGBMinGain(static_cast<float>(minimumGain * 128)) : m_cam->setStereoMinGain(static_cast<float>(minimumGain * 128)))) {
        return false;
    }
    if (!CheckCommand(color ? "setRGBAutoExpo" : "setStereoAutoExpo",
            color ? m_cam->setRGBAutoExpo(automatic ? 1 : 0) : m_cam->setStereoAutoExpo(automatic ? 1 : 0))) {
        return false;
    }
    if (!automatic) {
        const int exposure = color ? cfg.colorExposureUs : cfg.stereoExposureUs;
        const float gain = static_cast<float>((color ? cfg.colorGainX : cfg.stereoGainX) * 128);
        if (!CheckCommand(color ? "setRGBExposure" : "setStereoExposure",
                color ? m_cam->setRGBExposure(static_cast<unsigned>(exposure)) : m_cam->setStereoExposure(static_cast<unsigned>(exposure))) ||
            !CheckCommand(color ? "setRGBGain" : "setStereoGain", color ? m_cam->setRGBGain(gain) : m_cam->setStereoGain(gain))) {
            return false;
        }
    }
    return true;
}

bool CameraSession::WaitFirstFrame(int totalMs)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(totalMs);
    while (std::chrono::steady_clock::now() < deadline) {
        movesense::MovesenseFrame frame;
        const auto start = std::chrono::steady_clock::now();
        const int remaining = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - start).count());
        if (remaining <= 0) {
            break;
        }
        const int timeout = std::min(remaining, 200);
        if (m_cam->getFrame(frame, timeout)) {
            Log(LogLevel::Info, "first image frame received");
            return true;
        }
        std::vector<movesense::Imu> probe;
        if (m_cam->getIMU(probe, 0) < 0) {
            m_dataClosed = true;
            Log(LogLevel::Error, "data connection closed while waiting for the first image");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool CameraSession::WaitFirstImu(int totalMs)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(totalMs);
    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<movesense::Imu> samples;
        if (m_cam->getIMU(samples) < 0) {
            m_dataClosed = true;
            Log(LogLevel::Error, "data connection closed while waiting for the first IMU sample");
            return false;
        }
        for (const movesense::Imu& sample : samples) {
            if (sample.timestampUs != 0) {
                Log(LogLevel::Info, "first IMU sample received");
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool CameraSession::Init(const CameraConfig& cfg, LogSink log)
{
    m_log = std::move(log);
    if (m_cam != nullptr) {
        Log(LogLevel::Error, "camera session is already initialized");
        return false;
    }
    if (!ValidateConfig(cfg)) {
        return false;
    }
    m_cam = new Simou3Camera(cfg.cameraIp);
    if (!CheckCommand("openCameraSettings", m_cam->openCameraSettings())) {
        Shutdown(true);
        return false;
    }
    m_settingsOpened = true;
    if (!CheckCommand("getCameraType", m_cam->getCameraType(m_cameraType)) || m_cameraType < 0 || m_cameraType > 2) {
        Log(LogLevel::Error, "camera type is unavailable or unsupported");
        Shutdown(true);
        return false;
    }
    m_isPassive = m_cameraType == 1;
    const bool colorActive = cfg.enableColor && !m_isPassive;
    if (m_isPassive && cfg.enableColor) {
        Log(LogLevel::Warning, "passive camera has no RGB lens; color stream is disabled");
    }
    if (cfg.triggerMode >= 0 && !CheckCommand("setTriggerMode", m_cam->setTriggerMode(cfg.triggerMode))) {
        Shutdown(true);
        return false;
    }
    if (!CheckCommand("getTriggerMode", m_cam->getTriggerMode(m_triggerMode)) ||
        (m_triggerMode != movesense::kTriggerModeAuto && m_triggerMode != movesense::kTriggerModeExternal) ||
        (cfg.triggerMode >= 0 && cfg.triggerMode != m_triggerMode)) {
        Log(LogLevel::Error, "unsupported trigger mode; automatic or external mode is required and soft triggering is not provided");
        Shutdown(true);
        return false;
    }
    const int stereoDown = ResToDown(cfg.stereoWidth, cfg.stereoHeight);
    const int depthDown = ResToDown(cfg.depthWidth, cfg.depthHeight);
    const int colorDown = ResToDown(cfg.colorWidth, cfg.colorHeight);
    const int mode = cfg.downsampleMode < 0 ? ((stereoDown == 0 && depthDown == 1) ? 1 : 0) : cfg.downsampleMode;
    if (!CheckCommand("setDownsampleMode", m_cam->setDownsampleMode(mode)) ||
        !CheckCommand("setStereoDownsample", m_cam->setStereoDownsample(stereoDown == 1)) ||
        !CheckCommand("setDepthDownsample", m_cam->setDepthDownsample(depthDown == 1)) ||
        (colorActive && !CheckCommand("setRGBDownsample", m_cam->setRGBDownsample(colorDown == 1))) || !ApplyRoi(cfg) ||
        (!m_isPassive && cfg.registration >= 0 && !CheckCommand("setRegistrationSwitch", m_cam->setRegistrationSwitch(cfg.registration == 1))) ||
        !CheckCommand("setFrameRate", m_cam->setFrameRate(cfg.fps))) {
        Shutdown(true);
        return false;
    }
    if (cfg.enableDepth &&
        (!CheckCommand("getDepthUnitMm", m_cam->getDepthUnitMm(m_depthUnitMm)) || !std::isfinite(m_depthUnitMm) || m_depthUnitMm <= 0.0)) {
        Log(LogLevel::Error, "depth unit must be finite and positive");
        Shutdown(true);
        return false;
    }
    unsigned mask = 0;
    if (cfg.enableStereo) {
        mask |= (1u << TRANSFER_MODE_L_RECTIFIED_BIT) | (1u << TRANSFER_MODE_R_RECTIFIED_BIT);
    }
    if (cfg.enableDepth) {
        mask |= 1u << TRANSFER_MODE_DEPTH_BIT;
    }
    if (colorActive) {
        mask |= 1u << TRANSFER_MODE_RGB_RECTIFIED_BIT;
    }
    if (cfg.enableDetection) {
        mask |= 1u << TRANSFER_MODE_SEG_BIT;
    }
    if (cfg.enableImu) {
        mask |= 1u << TRANSFER_MODE_IMU_BIT;
    }
    if (mask == 0 || !CheckCommand("openCamera", m_cam->openCamera(static_cast<int>(mask)))) {
        Log(LogLevel::Error, "no supported stream enabled or opening the data stream failed");
        Shutdown(true);
        return false;
    }
    m_streaming = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!ApplyExposure(cfg, false) || (colorActive && !ApplyExposure(cfg, true)) ||
        (!m_isPassive && cfg.doePower >= 0 && !CheckCommand("setDOEPower", m_cam->setDOEPower(static_cast<unsigned>(cfg.doePower))))) {
        Shutdown(true);
        return false;
    }
    int actualFps = 0;
    if (!CheckCommand("getFrameRate", m_cam->getFrameRate(actualFps)) || actualFps < 5 || actualFps > 25) {
        Log(LogLevel::Error, "effective FPS is invalid");
        Shutdown(true);
        return false;
    }
    if (actualFps != cfg.fps) {
        Log(LogLevel::Warning, "requested FPS=" + std::to_string(cfg.fps) + ", effective FPS=" + std::to_string(actualFps));
    }
    const bool hasImages = (mask & ~(1u << TRANSFER_MODE_IMU_BIT)) != 0;
    if (hasImages && !WaitFirstFrame(cfg.firstFrameWaitMs)) {
        Log(LogLevel::Error, m_dataClosed ? "data connection failed while waiting for calibration readiness"
                                          : (m_triggerMode == movesense::kTriggerModeExternal
                                                    ? "external mode requires an input pulse before the startup timeout to read valid calibration"
                                                    : "automatic mode did not produce an image before the startup timeout"));
        Shutdown(true);
        return false;
    }
    if (!hasImages && cfg.enableImu && !WaitFirstImu(cfg.firstFrameWaitMs)) {
        Log(LogLevel::Error, "IMU startup timeout or data connection failure");
        Shutdown(true);
        return false;
    }
    if (cfg.alignTimeOnStart) {
        if (!CheckCommand("alignTimeToHost", m_cam->alignTimeToHost())) {
            Shutdown(true);
            return false;
        }
        const uint64_t nowUs =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        m_minimumTimestampUs = nowUs > 5000000 ? nowUs - 5000000 : 0;
        std::vector<movesense::Imu> discardedImu;
        if (m_cam->getIMU(discardedImu, INT_MAX) < 0) {
            Log(LogLevel::Error, "data connection closed while clearing pre-alignment IMU samples");
            Shutdown(true);
            return false;
        }
        movesense::MovesenseFrame discardedFrame;
        m_cam->getFrame(discardedFrame, 0);
    }
    if (!m_connectionProbe.Init(cfg.cameraIp, m_log, CameraConnectionProbe::ProbeFunction(), 2000, cfg.connectionProbe)) {
        Shutdown(true);
        return false;
    }
    std::ostringstream summary;
    summary << "connected " << cfg.cameraIp << " type=" << m_cameraType << " trigger_mode=" << m_triggerMode << " fps=" << actualFps
            << " downsample_mode=" << mode << " depth_unit_mm=" << m_depthUnitMm << " time_domain=" << (cfg.alignTimeOnStart ? "host" : "device");
    Log(LogLevel::Info, summary.str());
    return true;
}

void CameraSession::Shutdown(bool connectionLost)
{
    m_connectionProbe.Shutdown();
    if (m_cam == nullptr) {
        return;
    }
    bool disconnected = connectionLost || m_dataClosed || m_connectionProbe.Failed();
    if (!disconnected && m_streaming) {
        std::vector<movesense::Imu> probe;
        disconnected = m_cam->getIMU(probe, 0) < 0;
    }
    if (m_settingsOpened && m_roiActive && !disconnected) {
        for (int i = 0; i < kRoiStreamCount; ++i) {
            if (m_cam->setRoi(static_cast<movesense::RoiStream>(i), false, 0, 0, 0, 0) <= 0) {
                Log(LogLevel::Warning, std::string("ROI reset failed for ") + RoiStreamName(i));
                break;
            }
        }
    }
    if (m_streaming) {
        m_cam->closeCamera();
        m_streaming = false;
    }
    if (m_settingsOpened) {
        m_cam->closeCameraSettings();
    }
    delete m_cam;
    m_cam = nullptr;
    m_settingsOpened = false;
    m_roiActive = false;
    m_dataClosed = false;
    m_isPassive = false;
    m_cameraType = -1;
    m_depthUnitMm = 0.0;
    m_triggerMode = -1;
    m_minimumTimestampUs = 0;
}

bool CameraConnectionProbe::Failed() const
{
    return m_connectionFailed.load();
}

movesense::Simou3Camera* CameraSession::Camera()
{
    return m_cam;
}

bool CameraSession::IsPassive() const
{
    return m_isPassive;
}

int CameraSession::CameraType() const
{
    return m_cameraType;
}

double CameraSession::DepthUnitMm() const
{
    return m_depthUnitMm;
}

int CameraSession::TriggerMode() const
{
    return m_triggerMode;
}

uint64_t CameraSession::MinimumTimestampUs() const
{
    return m_minimumTimestampUs;
}

bool CameraSession::Failed() const
{
    return m_connectionProbe.Failed();
}

} // namespace movesense_x95
