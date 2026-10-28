// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95/calibration_export.h"
#include "movesense_x95/camera_config.h"
#include "movesense_x95/session_supervisor.h"
#include "movesense_x95/thread_safe_queue.h"
#include "movesense_x95_ros/camera_driver.h"
#include "movesense_x95_ros/imu_publisher.h"
#include "movesense_x95_ros/imu_receiver.h"
#include "movesense_x95_ros/stereo_calib.h"
#include "movesense_x95_ros/stereo_publisher.h"
#include "movesense_x95_ros/stereo_receiver.h"
#include "movesense_x95_ros/tf_publisher.h"

#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <stdexcept>
#include <string>
#include <thread>

using namespace movesense_x95_ros;
using namespace movesense_x95;

template<typename T>
static T ReadParameter(ros::NodeHandle& pnh, const std::string& name, const T& value)
{
    T result = value;
    pnh.param<T>(name, result, value);
    return result;
}

static CameraConfig LoadConfig(ros::NodeHandle& pnh)
{
    CameraConfig cfg;
    cfg.cameraIp = ReadParameter<std::string>(pnh, "camera_ip", cfg.cameraIp);
    cfg.alignTimeOnStart = ReadParameter<bool>(pnh, "align_time_on_start", cfg.alignTimeOnStart);
    cfg.connectionProbe = ReadParameter<bool>(pnh, "connection_probe", cfg.connectionProbe);
    cfg.triggerMode = ReadParameter<int>(pnh, "trigger_mode", cfg.triggerMode);
    cfg.firstFrameWaitMs = ReadParameter<int>(pnh, "first_frame_wait_ms", cfg.firstFrameWaitMs);
    cfg.fps = ReadParameter<int>(pnh, "depth_fps", cfg.fps);
    cfg.stereoWidth = ReadParameter<int>(pnh, "stereo_width", cfg.stereoWidth);
    cfg.stereoHeight = ReadParameter<int>(pnh, "stereo_height", cfg.stereoHeight);
    cfg.downsampleMode = ReadParameter<int>(pnh, "downsample_mode", cfg.downsampleMode);
    cfg.depthWidth = ReadParameter<int>(pnh, "depth_width", cfg.depthWidth);
    cfg.depthHeight = ReadParameter<int>(pnh, "depth_height", cfg.depthHeight);
    cfg.colorWidth = ReadParameter<int>(pnh, "color_width", cfg.colorWidth);
    cfg.colorHeight = ReadParameter<int>(pnh, "color_height", cfg.colorHeight);
    cfg.enableStereo = ReadParameter<bool>(pnh, "enable_stereo", cfg.enableStereo);
    cfg.enableDepth = ReadParameter<bool>(pnh, "enable_depth", cfg.enableDepth);
    cfg.enableColor = ReadParameter<bool>(pnh, "enable_color", cfg.enableColor);
    cfg.enableImu = ReadParameter<bool>(pnh, "enable_imu", cfg.enableImu);
    cfg.enableDetection = ReadParameter<bool>(pnh, "enable_detection", cfg.enableDetection);
    cfg.stereoAutoExpo = ReadParameter<bool>(pnh, "stereo_auto_expo", cfg.stereoAutoExpo);
    cfg.stereoExposureUs = ReadParameter<int>(pnh, "stereo_exposure_us", cfg.stereoExposureUs);
    cfg.stereoGainX = ReadParameter<int>(pnh, "stereo_gain_x", cfg.stereoGainX);
    cfg.stereoMaxExposureUs = ReadParameter<int>(pnh, "stereo_max_exposure_us", cfg.stereoMaxExposureUs);
    cfg.stereoMinExposureUs = ReadParameter<int>(pnh, "stereo_min_exposure_us", cfg.stereoMinExposureUs);
    cfg.stereoMaxGainX = ReadParameter<int>(pnh, "stereo_max_gain_x", cfg.stereoMaxGainX);
    cfg.stereoMinGainX = ReadParameter<int>(pnh, "stereo_min_gain_x", cfg.stereoMinGainX);
    cfg.colorAutoExpo = ReadParameter<bool>(pnh, "color_auto_expo", cfg.colorAutoExpo);
    cfg.colorExposureUs = ReadParameter<int>(pnh, "color_exposure_us", cfg.colorExposureUs);
    cfg.colorGainX = ReadParameter<int>(pnh, "color_gain_x", cfg.colorGainX);
    cfg.colorMaxExposureUs = ReadParameter<int>(pnh, "color_max_exposure_us", cfg.colorMaxExposureUs);
    cfg.colorMinExposureUs = ReadParameter<int>(pnh, "color_min_exposure_us", cfg.colorMinExposureUs);
    cfg.colorMaxGainX = ReadParameter<int>(pnh, "color_max_gain_x", cfg.colorMaxGainX);
    cfg.colorMinGainX = ReadParameter<int>(pnh, "color_min_gain_x", cfg.colorMinGainX);
    cfg.doePower = ReadParameter<int>(pnh, "doe_power", cfg.doePower);
    cfg.registration = ReadParameter<int>(pnh, "registration", cfg.registration);
    cfg.imuCalibDir = ReadParameter<std::string>(pnh, "imu_calib_dir", cfg.imuCalibDir);
    cfg.cam0Topic = ReadParameter<std::string>(pnh, "cam0_topic", cfg.cam0Topic);
    cfg.cam1Topic = ReadParameter<std::string>(pnh, "cam1_topic", cfg.cam1Topic);
    cfg.colorTopic = ReadParameter<std::string>(pnh, "color_topic", cfg.colorTopic);
    cfg.colorInfoTopic = ReadParameter<std::string>(pnh, "color_info_topic", cfg.colorInfoTopic);
    cfg.depthTopic = ReadParameter<std::string>(pnh, "depth_topic", cfg.depthTopic);
    cfg.depthInfoTopic = ReadParameter<std::string>(pnh, "depth_info_topic", cfg.depthInfoTopic);
    cfg.detTopic = ReadParameter<std::string>(pnh, "det_topic", cfg.detTopic);
    cfg.imuTopic = ReadParameter<std::string>(pnh, "imu_topic", cfg.imuTopic);
    cfg.publishTf = ReadParameter<bool>(pnh, "publish_tf", cfg.publishTf);
    cfg.baseFrameId = ReadParameter<std::string>(pnh, "base_frame_id", cfg.baseFrameId);
    cfg.cam0FrameId = ReadParameter<std::string>(pnh, "cam0_frame_id", cfg.cam0FrameId);
    cfg.cam1FrameId = ReadParameter<std::string>(pnh, "cam1_frame_id", cfg.cam1FrameId);
    cfg.colorFrameId = ReadParameter<std::string>(pnh, "color_frame_id", cfg.colorFrameId);
    cfg.detFrameId = ReadParameter<std::string>(pnh, "det_frame_id", cfg.detFrameId);
    cfg.imuFrameId = ReadParameter<std::string>(pnh, "imu_frame_id", cfg.imuFrameId);

    for (int i = 0; i < kRoiStreamCount; ++i) {
        const std::string prefix = std::string("roi_") + RoiStreamName(i) + "_";
        RoiConfig& roi = cfg.roi[i];
        roi.enable = ReadParameter<bool>(pnh, prefix + "enable", roi.enable);
        roi.x1 = ReadParameter<int>(pnh, prefix + "x1", roi.x1);
        roi.y1 = ReadParameter<int>(pnh, prefix + "y1", roi.y1);
        roi.x2 = ReadParameter<int>(pnh, prefix + "x2", roi.x2);
        roi.y2 = ReadParameter<int>(pnh, prefix + "y2", roi.y2);
    }
    return cfg;
}

static sensor_msgs::CameraInfo MakeCameraInfo(double fx, double fy, double cx, double cy, int width, int height, const std::string& frameId)
{
    sensor_msgs::CameraInfo info;
    info.header.frame_id = frameId;
    info.width = static_cast<uint32_t>(width);
    info.height = static_cast<uint32_t>(height);
    info.distortion_model = "plumb_bob";
    info.D.assign(5, 0.0);

    const double scaleX = static_cast<double>(width) / 1280.0;
    const double scaleY = static_cast<double>(height) / 960.0;
    info.K[0] = fx * scaleX;
    info.K[2] = cx * scaleX;
    info.K[4] = fy * scaleY;
    info.K[5] = cy * scaleY;
    info.K[8] = 1.0;
    info.R[0] = 1.0;
    info.R[4] = 1.0;
    info.R[8] = 1.0;
    info.P[0] = info.K[0];
    info.P[2] = info.K[2];
    info.P[5] = info.K[4];
    info.P[6] = info.K[5];
    info.P[10] = 1.0;
    return info;
}

static SessionResult RunCameraSession(ros::NodeHandle& nh, const CameraConfig& cfg, uint64_t attemptId)
{
    CameraDriver driver;
    StereoCalib calib;
    StereoFrameQueue frameQueue(static_cast<size_t>(cfg.frameQueueSize));
    ImuQueue imuQueue(static_cast<size_t>(cfg.imuQueueSize));
    StereoPublisher stereoPublisher;
    ImuPublisher imuPublisher;
    StereoReceiver stereoReceiver;
    ImuReceiver imuReceiver;
    TfPublisher tfPublisher;
    SessionResult result = SessionResult::Retry;

    try {
        if (!driver.Init(cfg)) {
            throw std::runtime_error("Camera initialization failed");
        }
        if (!calib.Init(driver.Camera())) {
            ROS_WARN("Stereo calibration is invalid; stereo CameraInfo/TF geometry is unavailable");
        }

        const bool colorActive = cfg.enableColor && !driver.IsPassive();
        const bool depthRegistered = cfg.enableDepth && cfg.registration == 1 && !driver.IsPassive();
        const bool needColorCalibration = colorActive || depthRegistered || cfg.enableDetection ||
                                          (cfg.enableImu && (driver.CameraType() == 0 || !calib.HasImuExtrinsic())) || !cfg.imuCalibDir.empty();
        bool colorCalibOk = false;
        if (needColorCalibration && !driver.IsPassive()) {
            colorCalibOk = calib.InitColor(driver.Camera()) && calib.HasColor();
        }

        calib.LogSummary();
        std::string exportError;
        if (!WriteCalibrationXml(calib, cfg.imuCalibDir, driver.CameraType(), exportError)) {
            throw std::runtime_error("Calibration XML export failed: " + exportError);
        }
        if (depthRegistered && (!colorCalibOk || !calib.HasColorExtrinsic())) {
            throw std::runtime_error("Registered depth requires valid RGB calibration");
        }
        if (!cfg.imuCalibDir.empty()) {
            ROS_INFO("Calibration XML written to %s", cfg.imuCalibDir.c_str());
        }

        const std::string detFrameId =
            cfg.detFrameId.empty() || cfg.detFrameId == "auto" ? (driver.IsPassive() ? cfg.cam0FrameId : cfg.colorFrameId) : cfg.detFrameId;
        const std::string depthFrameId = depthRegistered ? cfg.colorFrameId : cfg.cam0FrameId;
        stereoPublisher.SetImageGeometry(cfg, driver.IsPassive());
        if (cfg.enableDepth && !stereoPublisher.SetDepthUnitMm(driver.DepthUnitMm())) {
            throw std::runtime_error("Invalid depth unit");
        }
        if (cfg.enableDepth && (depthRegistered || calib.IsValid())) {
            const sensor_msgs::CameraInfo depthInfo =
                depthRegistered ? MakeCameraInfo(calib.ColorFx(), calib.ColorFy(), calib.ColorCx(), calib.ColorCy(), cfg.depthWidth, cfg.depthHeight,
                                      depthFrameId)
                                : MakeCameraInfo(calib.Fx(), calib.Fy(), calib.Cx(), calib.Cy(), cfg.depthWidth, cfg.depthHeight, depthFrameId);
            stereoPublisher.SetDepthCameraInfo(nh, cfg.depthInfoTopic, depthInfo, cfg.frameQueueSize);
        }
        if (colorActive && colorCalibOk) {
            const sensor_msgs::CameraInfo colorInfo =
                MakeCameraInfo(calib.ColorFx(), calib.ColorFy(), calib.ColorCx(), calib.ColorCy(), cfg.colorWidth, cfg.colorHeight, cfg.colorFrameId);
            stereoPublisher.SetColorCameraInfo(nh, cfg.colorInfoTopic, colorInfo, cfg.frameQueueSize);
        }
        if (cfg.publishTf) {
            bool imuOnColor = false;
            const bool imuCalibrationAvailable = calib.SelectImuCalibration(driver.CameraType(), imuOnColor);
            const bool colorFrameUsed = colorActive || depthRegistered || (!driver.IsPassive() && cfg.enableDetection) ||
                                        (cfg.enableImu && imuCalibrationAvailable && imuOnColor);
            tfPublisher.Publish(cfg, calib, colorFrameUsed, driver.IsPassive(), driver.CameraType());
        }

        if (!stereoPublisher.Init(nh, &frameQueue, cfg.cam0Topic, cfg.cam1Topic, cfg.colorTopic, cfg.depthTopic, cfg.detTopic, cfg.cam0FrameId,
                cfg.cam1FrameId, cfg.colorFrameId, depthFrameId, detFrameId, cfg.frameQueueSize, cfg.enableStereo, colorActive, cfg.enableDetection,
                cfg.enableDepth) ||
            !stereoReceiver.Init(driver.Camera(), &frameQueue, cfg.frameTimeoutMs, driver.MinimumTimestampUs())) {
            throw std::runtime_error("Image thread initialization failed");
        }
        if (cfg.enableImu && (!imuPublisher.Init(nh, &imuQueue, cfg.imuTopic, cfg.imuFrameId, cfg.imuQueueSize) ||
                                 !imuReceiver.Init(driver.Camera(), &imuQueue, cfg.imuPollMs, driver.MinimumTimestampUs()))) {
            throw std::runtime_error("IMU thread initialization failed");
        }

        ROS_INFO("Camera session %llu is publishing", static_cast<unsigned long long>(attemptId));
        while (ros::ok() && !driver.Failed() && !stereoReceiver.Failed() && !imuReceiver.Failed()) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        result = ros::ok() ? SessionResult::Retry : SessionResult::Stopped;
        if (result == SessionResult::Retry) {
            ROS_WARN("Camera connection was lost; stopping this session before reconnecting");
        }
    } catch (const std::exception& exception) {
        if (ros::ok()) {
            ROS_WARN("Camera session %llu unavailable: %s", static_cast<unsigned long long>(attemptId), exception.what());
        }
        result = ros::ok() ? SessionResult::Retry : SessionResult::Stopped;
    } catch (...) {
        ROS_ERROR("Unexpected camera session failure");
        result = SessionResult::Fatal;
    }

    const bool connectionLost = driver.Failed() || stereoReceiver.Failed() || imuReceiver.Failed();
    stereoPublisher.Shutdown();
    imuPublisher.Shutdown();
    stereoReceiver.Shutdown();
    imuReceiver.Shutdown();
    driver.Shutdown(connectionLost);
    calib.Shutdown();

    ROS_INFO("Camera session %llu stopped (frames dropped %llu, IMU dropped %llu)", static_cast<unsigned long long>(attemptId),
        static_cast<unsigned long long>(frameQueue.Dropped()), static_cast<unsigned long long>(imuQueue.Dropped()));
    return result;
}

int main(int argc, char** argv)
{
    if (argc == 2 && std::string(argv[1]) == "--version") {
        std::cout << MOVESENSE_X95_RELEASE_VERSION << std::endl;
        return 0;
    }
    ros::init(argc, argv, "movesense_x95_ros");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int exitCode = 1;
    SessionSupervisor supervisor;
    try {
        const CameraConfig cfg = LoadConfig(pnh);
        ROS_INFO("MoveSense X95 ROS release %s", MOVESENSE_X95_RELEASE_VERSION);
        if (ros::Time::isSimTime()) {
            throw std::runtime_error("Live camera timestamps require use_sim_time=false");
        }
        CameraDriver validator;
        if (!validator.ValidateConfiguration(cfg)) {
            throw std::runtime_error("Invalid camera configuration");
        }
        const auto runSession = [&nh, &cfg](uint64_t attemptId) { return RunCameraSession(nh, cfg, attemptId); };
        const auto running = []() { return ros::ok(); };
        const auto spin = []() { ros::spinOnce(); };
        const auto logger = [](LogLevel level, const std::string& message) {
            if (level == LogLevel::Error) {
                ROS_ERROR("%s", message.c_str());
            } else if (level == LogLevel::Warning) {
                ROS_WARN("%s", message.c_str());
            } else {
                ROS_INFO("%s", message.c_str());
            }
        };
        if (!supervisor.Init(runSession, running, spin, logger, 1000)) {
            throw std::runtime_error("Camera session supervisor initialization failed");
        }
        exitCode = supervisor.Run();
    } catch (const std::exception& exception) {
        ROS_ERROR("%s", exception.what());
    }
    supervisor.Shutdown();
    ros::shutdown();
    return exitCode;
}
