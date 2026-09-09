// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All
// rights reserved.

#include "movesense_x95_ros/camera_driver.h"

#include <chrono>
#include <movesense/Simou3Camera.h>
#include <movesense/transfer_mode_def.h>
#include <ros/ros.h>

using namespace movesense;

namespace movesense_x95_ros {

static int ResToDown(int w, int h)
{
    if (w == 1280 && h == 960) {
        return 0;
    }

    if (w == 640 && h == 480) {
        return 1;
    }

    return -1;
}

static float GainXToRaw(int gainX)
{
    if (gainX < 1) {
        gainX = 1;
    }
    return static_cast<float>(gainX * 128);
}

static void ApplyStereoSettings(Simou3Camera* cam, const CameraConfig& cfg)
{
    if (cfg.stereoMaxExposureUs >= 0) {
        cam->setStereoMaxExposure(static_cast<unsigned>(cfg.stereoMaxExposureUs));
    }

    if (cfg.stereoMinExposureUs >= 0) {
        cam->setStereoMinExposure(static_cast<unsigned>(cfg.stereoMinExposureUs));
    }

    if (cfg.stereoMaxGainX >= 0) {
        cam->setStereoMaxGain(GainXToRaw(cfg.stereoMaxGainX));
    }

    if (cfg.stereoMinGainX >= 0) {
        cam->setStereoMinGain(GainXToRaw(cfg.stereoMinGainX));
    }

    cam->setStereoAutoExpo(cfg.stereoAutoExpo ? 1 : 0);
    if (!cfg.stereoAutoExpo) {
        cam->setStereoExposure(static_cast<unsigned>(cfg.stereoExposureUs));
        cam->setStereoGain(GainXToRaw(cfg.stereoGainX));
    }
}

static void ApplyColorSettings(Simou3Camera* cam, const CameraConfig& cfg)
{
    if (cfg.colorMaxExposureUs >= 0) {
        cam->setRGBMaxExposure(static_cast<unsigned>(cfg.colorMaxExposureUs));
    }

    if (cfg.colorMinExposureUs >= 0) {
        cam->setRGBMinExposure(static_cast<unsigned>(cfg.colorMinExposureUs));
    }

    if (cfg.colorMaxGainX >= 0) {
        cam->setRGBMaxGain(GainXToRaw(cfg.colorMaxGainX));
    }

    if (cfg.colorMinGainX >= 0) {
        cam->setRGBMinGain(GainXToRaw(cfg.colorMinGainX));
    }

    cam->setRGBAutoExpo(cfg.colorAutoExpo ? 1 : 0);
    if (!cfg.colorAutoExpo) {
        cam->setRGBExposure(static_cast<unsigned>(cfg.colorExposureUs));
        cam->setRGBGain(GainXToRaw(cfg.colorGainX));
    }
}

bool CameraDriver::WaitFirstFrame(int totalMs)
{
    if (totalMs <= 0) {
        return true;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(totalMs);
    const auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() < deadline) {
        MovesenseFrame frame;
        if (m_cam->getFrame(frame, 200)) {
            const long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
            ROS_INFO("[CameraDriver] first frame received after %lld ms", ms);
            return true;
        }
    }

    return false;
}

CameraDriver::CameraDriver() {}

CameraDriver::~CameraDriver() {}

bool CameraDriver::Init(const CameraConfig& cfg)
{
    m_cam = new Simou3Camera(cfg.cameraIp);

    if (m_cam->openCameraSettings() <= 0) {
        ROS_ERROR("[CameraDriver] openCameraSettings failed (ip=%s)", cfg.cameraIp.c_str());
        Shutdown();
        return false;
    }
    m_settingsOpened = true;

    m_cameraType = 0;
    m_cam->getCameraType(m_cameraType);
    m_isPassive = (m_cameraType == 1);
    const char* typeName = (m_cameraType == 1) ? "P" : (m_cameraType == 2) ? "AP" : "A";

    if (m_isPassive) {
        if (cfg.enableColor) {
            ROS_WARN("[CameraDriver] passive (P) camera has no RGB lens; "
                     "enable_color ignored");
        }
        if (cfg.registration > 0) {
            ROS_WARN("[CameraDriver] passive (P) camera has no color stream; "
                     "registration ignored");
        }
        if (cfg.doePower >= 0) {
            ROS_WARN("[CameraDriver] passive (P) camera has no DOE projector; "
                     "doe_power ignored");
        }
    }

    const int sd = ResToDown(cfg.stereoWidth, cfg.stereoHeight);
    const int dd = ResToDown(cfg.depthWidth, cfg.depthHeight);
    const int cd = ResToDown(cfg.colorWidth, cfg.colorHeight);

    if (sd < 0 || dd < 0 || cd < 0) {
        ROS_ERROR("[CameraDriver] unsupported resolution (only 1280x960 or "
                  "640x480): stereo=%dx%d depth=%dx%d color=%dx%d",
            cfg.stereoWidth, cfg.stereoHeight, cfg.depthWidth, cfg.depthHeight, cfg.colorWidth, cfg.colorHeight);
        Shutdown();
        return false;
    }

    if (sd == 1 && dd == 0) {
        ROS_ERROR("[CameraDriver] stereo resolution (%dx%d) must be >= depth "
                  "resolution (%dx%d)",
            cfg.stereoWidth, cfg.stereoHeight, cfg.depthWidth, cfg.depthHeight);
        Shutdown();
        return false;
    }

    int mode = cfg.downsampleMode;

    if (mode < 0) {
        mode = (sd == 0 && dd == 1) ? 1 : 0;
    } else if (mode > 1) {
        ROS_ERROR("[CameraDriver] downsample_mode must be -1 (auto), 0 (pre) or 1 (post), got %d", mode);
        Shutdown();
        return false;
    } else if (sd == 0 && dd == 1 && mode == 0) {
        ROS_ERROR("[CameraDriver] stereo %dx%d with depth %dx%d requires downsample_mode=1 (post); "
                  "pre-downsampling would shrink the stereo pair as well",
            cfg.stereoWidth, cfg.stereoHeight, cfg.depthWidth, cfg.depthHeight);
        Shutdown();
        return false;
    } else if (sd == 0 && dd == 0) {
        ROS_WARN("[CameraDriver] neither stereo nor depth is downsampled; downsample_mode=%d has no effect", mode);
    }

    const bool colorActive = cfg.enableColor && !m_isPassive;

    m_cam->setFrameRate(cfg.fps);
    m_cam->setDownsampleMode(mode);
    m_cam->setStereoDownsample(sd == 1);
    m_cam->setDepthDownsample(dd == 1);
    if (colorActive) {
        m_cam->setRGBDownsample(cd == 1);
    }

    if (cfg.registration >= 0 && !m_isPassive) {
        m_cam->setRegistrationSwitch(cfg.registration != 0);
    }

    unsigned int mask = 0;
    if (cfg.enableStereo) {
        mask |= (1u << TRANSFER_MODE_L_RECTIFIED_BIT) | (1u << TRANSFER_MODE_R_RECTIFIED_BIT);
    }
    if (cfg.enableDepth) {
        mask |= (1u << TRANSFER_MODE_DEPTH_BIT);
    }
    if (colorActive) {
        mask |= (1u << TRANSFER_MODE_RGB_RECTIFIED_BIT);
    }
    if (cfg.enableDetection) {
        mask |= (1u << TRANSFER_MODE_SEG_BIT);
    }
    if (cfg.enableImu) {
        mask |= (1u << TRANSFER_MODE_IMU_BIT);
    }
    if (mask == 0) {
        ROS_ERROR("[CameraDriver] no stream enabled (enable at least one of "
                  "stereo/depth/color/imu)");
        Shutdown();
        return false;
    }

    if (m_cam->openCamera(static_cast<int>(mask)) <= 0) {
        ROS_ERROR("[CameraDriver] openCamera failed (ip=%s)", cfg.cameraIp.c_str());
        Shutdown();
        return false;
    }
    m_streaming = true;

    ros::Duration(0.5).sleep();

    ApplyStereoSettings(m_cam, cfg);
    if (colorActive) {
        ApplyColorSettings(m_cam, cfg);
    }
    if (cfg.doePower >= 0 && !m_isPassive) {
        m_cam->setDOEPower(static_cast<unsigned>(cfg.doePower));
    }

    if (cfg.alignTimeOnStart) {
        int rc = m_cam->alignTimeToHost();
        if (rc <= 0) {
            ROS_WARN("[CameraDriver] alignTimeToHost failed (rc=%d)", rc);
        } else {
            ROS_INFO("[CameraDriver] alignTimeToHost done");
        }
    }

    if (!WaitFirstFrame(cfg.firstFrameWaitMs)) {
        ROS_WARN("[CameraDriver] no frame within %d ms; calibration read may be unreliable", cfg.firstFrameWaitMs);
    }

    ROS_INFO("[CameraDriver] connected %s | type=%s(%d) passive=%d | fps=%d "
             "downsample_mode=%d(%s)",
        cfg.cameraIp.c_str(), typeName, m_cameraType, m_isPassive ? 1 : 0, cfg.fps, mode, mode ? "post" : "pre");
    ROS_INFO("[CameraDriver] res: stereo=%dx%d(down=%d) depth=%dx%d(down=%d) "
             "color=%dx%d(down=%d, active=%d) | streams: stereo=%d depth=%d color=%d "
             "imu=%d det=%d",
        cfg.stereoWidth, cfg.stereoHeight, sd, cfg.depthWidth, cfg.depthHeight, dd, cfg.colorWidth, cfg.colorHeight, cd, colorActive,
        cfg.enableStereo, cfg.enableDepth, colorActive, cfg.enableImu, cfg.enableDetection);

    return true;
}

void CameraDriver::Shutdown()
{
    if (m_cam == nullptr) {
        return;
    }

    if (m_streaming) {
        m_cam->closeCamera();
        m_streaming = false;
    }

    if (m_settingsOpened) {
        m_cam->closeCameraSettings();
        m_settingsOpened = false;
    }

    delete m_cam;
    m_cam = nullptr;
}

} // namespace movesense_x95_ros
