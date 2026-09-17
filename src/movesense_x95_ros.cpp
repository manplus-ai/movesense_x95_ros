// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/camera_config.h"
#include "movesense_x95_ros/camera_driver.h"
#include "movesense_x95_ros/imu_publisher.h"
#include "movesense_x95_ros/imu_receiver.h"
#include "movesense_x95_ros/stereo_calib.h"
#include "movesense_x95_ros/stereo_publisher.h"
#include "movesense_x95_ros/stereo_receiver.h"
#include "movesense_x95_ros/tf_publisher.h"
#include "movesense_x95_ros/thread_safe_queue.h"

#include <array>
#include <filesystem>
#include <chrono>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <thread>

using namespace movesense_x95_ros;

static CameraConfig LoadConfig(ros::NodeHandle& pnh)
{
    CameraConfig cfg;

    pnh.param<std::string>("camera_ip", cfg.cameraIp, cfg.cameraIp);
    pnh.param("align_time_on_start", cfg.alignTimeOnStart, cfg.alignTimeOnStart);
    pnh.param("first_frame_wait_ms", cfg.firstFrameWaitMs, cfg.firstFrameWaitMs);

    pnh.param("depth_fps", cfg.fps, cfg.fps);
    pnh.param("stereo_width", cfg.stereoWidth, cfg.stereoWidth);
    pnh.param("stereo_height", cfg.stereoHeight, cfg.stereoHeight);
    pnh.param("downsample_mode", cfg.downsampleMode, cfg.downsampleMode);
    pnh.param("depth_width", cfg.depthWidth, cfg.depthWidth);
    pnh.param("depth_height", cfg.depthHeight, cfg.depthHeight);
    pnh.param("color_width", cfg.colorWidth, cfg.colorWidth);
    pnh.param("color_height", cfg.colorHeight, cfg.colorHeight);

    pnh.param("enable_stereo", cfg.enableStereo, cfg.enableStereo);
    pnh.param("enable_depth", cfg.enableDepth, cfg.enableDepth);
    pnh.param("enable_color", cfg.enableColor, cfg.enableColor);
    pnh.param("enable_imu", cfg.enableImu, cfg.enableImu);
    pnh.param("enable_detection", cfg.enableDetection, cfg.enableDetection);

    pnh.param("stereo_auto_expo", cfg.stereoAutoExpo, cfg.stereoAutoExpo);
    pnh.param("stereo_exposure_us", cfg.stereoExposureUs, cfg.stereoExposureUs);
    pnh.param("stereo_gain_x", cfg.stereoGainX, cfg.stereoGainX);
    pnh.param("stereo_max_exposure_us", cfg.stereoMaxExposureUs, cfg.stereoMaxExposureUs);
    pnh.param("stereo_min_exposure_us", cfg.stereoMinExposureUs, cfg.stereoMinExposureUs);
    pnh.param("stereo_max_gain_x", cfg.stereoMaxGainX, cfg.stereoMaxGainX);
    pnh.param("stereo_min_gain_x", cfg.stereoMinGainX, cfg.stereoMinGainX);

    pnh.param("color_auto_expo", cfg.colorAutoExpo, cfg.colorAutoExpo);
    pnh.param("color_exposure_us", cfg.colorExposureUs, cfg.colorExposureUs);
    pnh.param("color_gain_x", cfg.colorGainX, cfg.colorGainX);
    pnh.param("color_max_exposure_us", cfg.colorMaxExposureUs, cfg.colorMaxExposureUs);
    pnh.param("color_min_exposure_us", cfg.colorMinExposureUs, cfg.colorMinExposureUs);
    pnh.param("color_max_gain_x", cfg.colorMaxGainX, cfg.colorMaxGainX);
    pnh.param("color_min_gain_x", cfg.colorMinGainX, cfg.colorMinGainX);

    pnh.param("doe_power", cfg.doePower, cfg.doePower);
    pnh.param("registration", cfg.registration, cfg.registration);

    pnh.param<std::string>("imu_calib_dir", cfg.imuCalibDir, cfg.imuCalibDir);

    pnh.param<std::string>("cam0_topic", cfg.cam0Topic, cfg.cam0Topic);
    pnh.param<std::string>("cam1_topic", cfg.cam1Topic, cfg.cam1Topic);
    pnh.param<std::string>("color_topic", cfg.colorTopic, cfg.colorTopic);
    pnh.param<std::string>("color_info_topic", cfg.colorInfoTopic, cfg.colorInfoTopic);
    pnh.param<std::string>("depth_topic", cfg.depthTopic, cfg.depthTopic);
    pnh.param<std::string>("depth_info_topic", cfg.depthInfoTopic, cfg.depthInfoTopic);
    pnh.param<std::string>("det_topic", cfg.detTopic, cfg.detTopic);
    pnh.param<std::string>("imu_topic", cfg.imuTopic, cfg.imuTopic);
    pnh.param("publish_tf", cfg.publishTf, cfg.publishTf);
    pnh.param<std::string>("base_frame_id", cfg.baseFrameId, cfg.baseFrameId);
    pnh.param<std::string>("cam0_frame_id", cfg.cam0FrameId, cfg.cam0FrameId);
    pnh.param<std::string>("cam1_frame_id", cfg.cam1FrameId, cfg.cam1FrameId);
    pnh.param<std::string>("color_frame_id", cfg.colorFrameId, cfg.colorFrameId);
    pnh.param<std::string>("det_frame_id", cfg.detFrameId, cfg.detFrameId);
    pnh.param<std::string>("imu_frame_id", cfg.imuFrameId, cfg.imuFrameId);

    for (int i = 0; i < kRoiStreamCount; ++i) {
        const std::string prefix = std::string("roi_") + RoiStreamName(i) + "_";
        RoiConfig& r = cfg.roi[i];
        pnh.param(prefix + "enable", r.enable, r.enable);
        pnh.param(prefix + "x1", r.x1, r.x1);
        pnh.param(prefix + "y1", r.y1, r.y1);
        pnh.param(prefix + "x2", r.x2, r.x2);
        pnh.param(prefix + "y2", r.y2, r.y2);
    }

    return cfg;
}

static sensor_msgs::CameraInfo MakeCameraInfo(
    double fx1280, double fy1280, double cx1280, double cy1280, int width, int height, const std::string& frameId)
{
    sensor_msgs::CameraInfo info;
    info.header.frame_id = frameId;
    info.width = static_cast<uint32_t>(width);
    info.height = static_cast<uint32_t>(height);
    info.distortion_model = "plumb_bob";
    info.D.assign(5, 0.0);

    const double s = static_cast<double>(width) / 1280.0;
    const double fx = fx1280 * s;
    const double fy = fy1280 * s;
    const double cx = cx1280 * s;
    const double cy = cy1280 * s;

    info.K[0] = fx;
    info.K[1] = 0.0;
    info.K[2] = cx;
    info.K[3] = 0.0;
    info.K[4] = fy;
    info.K[5] = cy;
    info.K[6] = 0.0;
    info.K[7] = 0.0;
    info.K[8] = 1.0;

    info.R[0] = 1.0;
    info.R[1] = 0.0;
    info.R[2] = 0.0;
    info.R[3] = 0.0;
    info.R[4] = 1.0;
    info.R[5] = 0.0;
    info.R[6] = 0.0;
    info.R[7] = 0.0;
    info.R[8] = 1.0;

    info.P[0] = fx;
    info.P[1] = 0.0;
    info.P[2] = cx;
    info.P[3] = 0.0;
    info.P[4] = 0.0;
    info.P[5] = fy;
    info.P[6] = cy;
    info.P[7] = 0.0;
    info.P[8] = 0.0;
    info.P[9] = 0.0;
    info.P[10] = 1.0;
    info.P[11] = 0.0;

    return info;
}

static bool WriteMat(const std::string& path, const char* name, const cv::Mat& m)
{
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }
    fs << name << m;
    fs.release();
    return true;
}

static cv::Mat Intrinsics3x3(const std::array<double, 4>& k)
{
    cv::Mat m = cv::Mat::zeros(3, 3, CV_64F);
    m.at<double>(0, 0) = k[0];
    m.at<double>(1, 1) = k[1];
    m.at<double>(0, 2) = k[2];
    m.at<double>(1, 2) = k[3];
    m.at<double>(2, 2) = 1.0;
    return m;
}

static cv::Mat Distortion8x1(const std::array<double, 8>& d)
{
    cv::Mat m = cv::Mat::zeros(8, 1, CV_64F);
    for (int i = 0; i < 8; ++i) {
        m.at<double>(i, 0) = d[i];
    }
    return m;
}

static cv::Mat Matrix3x3(const std::array<double, 9>& r)
{
    cv::Mat m(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        m.at<double>(i / 3, i % 3) = r[i];
    }
    return m;
}

static cv::Mat Column3x1(const std::array<double, 3>& t)
{
    cv::Mat m(3, 1, CV_64F);
    for (int i = 0; i < 3; ++i) {
        m.at<double>(i, 0) = t[i];
    }
    return m;
}

static bool WriteStereoGeometryXml(const StereoCalib& calib, const std::string& dir)
{
    cv::Mat p = cv::Mat::zeros(3, 4, CV_64F);
    p.at<double>(0, 0) = calib.Fx();
    p.at<double>(1, 1) = calib.Fy();
    p.at<double>(0, 2) = calib.Cx();
    p.at<double>(1, 2) = calib.Cy();
    p.at<double>(0, 3) = calib.NegBaselineMm() * calib.Fx();
    p.at<double>(2, 2) = 1.0;

    bool ok = true;
    ok = WriteMat(dir + "/M1.xml", "M1", Intrinsics3x3(calib.LeftIntrinsics())) && ok;
    ok = WriteMat(dir + "/D1.xml", "D1", Distortion8x1(calib.LeftDistortion())) && ok;
    ok = WriteMat(dir + "/M2.xml", "M2", Intrinsics3x3(calib.RightIntrinsics())) && ok;
    ok = WriteMat(dir + "/D2.xml", "D2", Distortion8x1(calib.RightDistortion())) && ok;
    ok = WriteMat(dir + "/IR_1.xml", "IR_1", Matrix3x3(calib.LeftRectRotation())) && ok;
    ok = WriteMat(dir + "/IR_2.xml", "IR_2", Matrix3x3(calib.RightRectRotation())) && ok;
    ok = WriteMat(dir + "/t_P2.xml", "t_P2", p) && ok;

    return ok;
}

static bool WriteColorGeometryXml(const StereoCalib& calib, const std::string& dir)
{
    cv::Mat p = cv::Mat::zeros(3, 3, CV_64F);
    p.at<double>(0, 0) = calib.ColorFx();
    p.at<double>(1, 1) = calib.ColorFy();
    p.at<double>(0, 2) = calib.ColorCx();
    p.at<double>(1, 2) = calib.ColorCy();
    p.at<double>(2, 2) = 1.0;

    bool ok = true;
    ok = WriteMat(dir + "/M2.xml", "M2", Intrinsics3x3(calib.ColorIntrinsics())) && ok;
    ok = WriteMat(dir + "/D2.xml", "D2", Distortion8x1(calib.ColorDistortion())) && ok;
    ok = WriteMat(dir + "/t_P2.xml", "t_P2", p) && ok;
    ok = WriteMat(dir + "/iR_2.xml", "iR_2", Matrix3x3(calib.ColorInverseRectification())) && ok;
    ok = WriteMat(dir + "/T_reg.xml", "T_reg", Column3x1(calib.ColorTregMm())) && ok;
    ok = WriteMat(dir + "/R.xml", "R", Matrix3x3(calib.ColorRotation())) && ok;
    ok = WriteMat(dir + "/T.xml", "T", Column3x1(calib.ColorTranslationMm())) && ok;

    return ok;
}

static bool WriteOneImuCalibXml(const std::string& dir, const std::array<double, 9>& r, const std::array<double, 3>& t, double tsMs,
    double gyroNd, double gyroRw, double accNd, double accRw)
{
    std::filesystem::create_directories(dir);

    bool ok = true;
    cv::FileStorage fs;

    fs.open(dir + "/noise_IMU.xml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "gyroscope_noise_density" << gyroNd;
        fs << "gyroscope_random_walk" << gyroRw;
        fs << "accelerometer_noise_density" << accNd;
        fs << "accelerometer_random_walk" << accRw;
        fs.release();
    } else {
        ok = false;
    }

    fs.open(dir + "/ts_IMU.xml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "timeshift_cam_imu" << tsMs;
        fs.release();
    } else {
        ok = false;
    }

    cv::Mat R(3, 3, CV_64F);
    for (int i = 0; i < 9; i++) {
        R.at<double>(i / 3, i % 3) = r[i];
    }
    fs.open(dir + "/r_IMU.xml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "r_IMU" << R;
        fs.release();
    } else {
        ok = false;
    }

    cv::Mat tv(1, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
        tv.at<double>(0, i) = t[i];
    }
    fs.open(dir + "/t_IMU.xml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "t_IMU" << tv;
        fs.release();
    } else {
        ok = false;
    }

    return ok;
}

static const char* CameraTypeName(int cameraType)
{
    if (cameraType == 1) {
        return "P";
    }
    if (cameraType == 2) {
        return "AP";
    }
    return "A";
}

static void WriteCalibXml(const StereoCalib& calib, const std::string& dir, int cameraType)
{
    if (dir.empty()) {
        return;
    }

    const char* typeName = CameraTypeName(cameraType);
    const bool imuOnColorOnly = (cameraType == 0);
    const bool hasColorLens = (cameraType != 1);

    if (!calib.HasStereoBlob()) {
        ROS_WARN("[NODE] calib export: stereo calibration could not be read from a %s camera; nothing written", typeName);
        return;
    }

    if (!calib.IsValid()) {
        ROS_WARN("[NODE] calib export: rectified intrinsics are invalid on a %s camera; the exported files reflect that", typeName);
    }

    try {
        const std::string sub = dir + "/xml";
        std::filesystem::create_directories(sub);

        if (WriteStereoGeometryXml(calib, sub)) {
            ROS_INFO("[NODE] stereo calib xml written to %s (M1 D1 M2 D2 IR_1 IR_2 t_P2)", sub.c_str());
        } else {
            ROS_ERROR("[NODE] stereo calib xml partially failed, dir=%s", sub.c_str());
        }

        if (calib.HasImuExtrinsic()) {
            if (WriteOneImuCalibXml(sub, calib.ImuRotation(), calib.ImuTranslationMm(), calib.ImuTimeshiftMs(), calib.GyroNoiseDensity(),
                    calib.GyroRandomWalk(), calib.AccelNoiseDensity(), calib.AccelRandomWalk())) {
                ROS_INFO("[NODE] IMU<->left calib xml written to %s", sub.c_str());
            } else {
                ROS_ERROR("[NODE] IMU<->left calib xml partially failed, dir=%s", sub.c_str());
            }
        } else if (imuOnColorOnly) {
            ROS_INFO("[NODE] a %s camera carries the IMU extrinsic against the color camera only; %s holds camera geometry", typeName,
                sub.c_str());
        } else {
            ROS_WARN("[NODE] IMU<->left extrinsic is empty on a %s camera, which should carry it; upload the stereo-channel IMU calibration",
                typeName);
        }

        if (!hasColorLens) {
            ROS_INFO("[NODE] a %s camera has no color lens; xml_rgb not written", typeName);
            return;
        }

        if (!calib.HasColor()) {
            ROS_WARN("[NODE] color calibration is unreadable on a %s camera, which should carry it; xml_rgb not written", typeName);
            return;
        }

        const std::string subRgb = dir + "/xml_rgb";
        std::filesystem::create_directories(subRgb);

        if (calib.HasColorExtrinsic()) {
            if (WriteColorGeometryXml(calib, subRgb)) {
                ROS_INFO("[NODE] color calib xml written to %s (M2 D2 t_P2 iR_2 T_reg R T)", subRgb.c_str());
            } else {
                ROS_ERROR("[NODE] color calib xml partially failed, dir=%s", subRgb.c_str());
            }
        } else {
            ROS_WARN("[NODE] color extrinsic is unusable on a %s camera, which should carry it; color geometry xml not written to %s", typeName,
                subRgb.c_str());
        }

        if (calib.HasColorImuExtrinsic()) {
            if (WriteOneImuCalibXml(subRgb, calib.ColorImuRotation(), calib.ColorImuTranslationMm(), calib.ColorImuTimeshiftMs(),
                    calib.ColorGyroNoiseDensity(), calib.ColorGyroRandomWalk(), calib.ColorAccelNoiseDensity(), calib.ColorAccelRandomWalk())) {
                ROS_INFO("[NODE] IMU<->color calib xml written to %s", subRgb.c_str());
            } else {
                ROS_ERROR("[NODE] IMU<->color calib xml partially failed, dir=%s", subRgb.c_str());
            }
        } else {
            ROS_WARN("[NODE] IMU<->color extrinsic is empty on a %s camera, which should carry it; upload the RGB-channel IMU calibration",
                typeName);
        }
    } catch (const std::exception& e) {
        ROS_ERROR("[NODE] calib xml write error(%s), dir=%s", e.what(), dir.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "movesense_x95_ros");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    CameraConfig cfg = LoadConfig(pnh);

    CameraDriver driver;
    if (!driver.Init(cfg)) {
        ROS_ERROR("[movesense_x95_ros] CameraDriver.Init failed, exiting");
        return 1;
    }

    StereoCalib calib;
    if (!calib.Init(driver.Camera())) {
        ROS_WARN("[movesense_x95_ros] no valid stereo extrinsics (camera may be uncalibrated); still publishing images/IMU");
    }

    ROS_INFO("[movesense_x95_ros] topics: left=%s right=%s depth=%s det=%s imu=%s (stereo mono8)", cfg.cam1Topic.c_str(), cfg.cam0Topic.c_str(),
        cfg.depthTopic.c_str(), cfg.detTopic.c_str(), cfg.imuTopic.c_str());

    StereoFrameQueue frameQueue(static_cast<size_t>(cfg.frameQueueSize));
    ImuQueue imuQueue(static_cast<size_t>(cfg.imuQueueSize));

    StereoPublisher stereoPublisher;
    ImuPublisher imuPublisher;
    StereoReceiver stereoReceiver;
    ImuReceiver imuReceiver;
    TfPublisher tfPublisher;

    const bool colorActive = cfg.enableColor && !driver.IsPassive();
    const bool depthRegistered = cfg.registration == 1 && !driver.IsPassive();

    bool colorCalibOk = false;
    if ((colorActive || depthRegistered || !cfg.imuCalibDir.empty()) && !driver.IsPassive()) {
        colorCalibOk = calib.InitColor(driver.Camera()) && calib.HasColor();
    }

    calib.LogSummary();
    WriteCalibXml(calib, cfg.imuCalibDir, driver.CameraType());

    const bool depthInColorFrame = depthRegistered && colorCalibOk;
    const std::string depthFrameId = depthInColorFrame ? cfg.colorFrameId : cfg.cam0FrameId;

    if (calib.IsValid()) {
        sensor_msgs::CameraInfo depthInfo;
        if (depthInColorFrame) {
            depthInfo =
                MakeCameraInfo(calib.ColorFx(), calib.ColorFy(), calib.ColorCx(), calib.ColorCy(), cfg.depthWidth, cfg.depthHeight, depthFrameId);
        } else {
            if (depthRegistered) {
                ROS_WARN("[movesense_x95_ros] registration is on but color calib is invalid; depth camera_info falls back to stereo intrinsics");
            } else if (cfg.registration == -1) {
                ROS_WARN("[movesense_x95_ros] registration=-1 (camera-side state unknown); depth camera_info uses stereo intrinsics");
            }
            depthInfo = MakeCameraInfo(calib.Fx(), calib.Fy(), calib.Cx(), calib.Cy(), cfg.depthWidth, cfg.depthHeight, depthFrameId);
        }
        stereoPublisher.SetDepthCameraInfo(nh, cfg.depthInfoTopic, depthInfo, cfg.frameQueueSize);
    }

    if (colorActive && colorCalibOk) {
        sensor_msgs::CameraInfo colorInfo =
            MakeCameraInfo(calib.ColorFx(), calib.ColorFy(), calib.ColorCx(), calib.ColorCy(), cfg.colorWidth, cfg.colorHeight, cfg.colorFrameId);
        stereoPublisher.SetColorCameraInfo(nh, cfg.colorInfoTopic, colorInfo, cfg.frameQueueSize);
    }

    if (cfg.publishTf) {
        tfPublisher.Publish(cfg, calib, colorActive || depthInColorFrame, driver.IsPassive(), driver.CameraType());
    }

    bool ok = true;
    ok = ok &&
         stereoPublisher.Init(nh, &frameQueue, cfg.cam0Topic, cfg.cam1Topic, cfg.colorTopic, cfg.depthTopic, cfg.detTopic, cfg.cam0FrameId,
             cfg.cam1FrameId, cfg.colorFrameId, depthFrameId, cfg.detFrameId, cfg.frameQueueSize, cfg.enableStereo, colorActive, cfg.enableDetection);
    ok = ok && stereoReceiver.Init(driver.Camera(), &frameQueue, cfg.frameTimeoutMs);
    if (cfg.enableImu) {
        ok = ok && imuPublisher.Init(nh, &imuQueue, cfg.imuTopic, cfg.imuFrameId, cfg.imuQueueSize);
        ok = ok && imuReceiver.Init(driver.Camera(), &imuQueue, cfg.imuPollMs);
    }

    if (!ok) {
        ROS_ERROR("[movesense_x95_ros] thread Init failed, exiting");

        stereoReceiver.Shutdown();
        imuReceiver.Shutdown();
        stereoPublisher.Shutdown();
        imuPublisher.Shutdown();
        driver.Shutdown();

        return 1;
    }

    ros::spin();

    ROS_INFO("[movesense_x95_ros] shutting down ...");

    std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        std::_Exit(0);
    }).detach();

    stereoReceiver.Shutdown();
    imuReceiver.Shutdown();

    stereoPublisher.Shutdown();
    imuPublisher.Shutdown();

    driver.Shutdown();

    ROS_INFO("[movesense_x95_ros] exited (frames dropped %llu, IMU dropped %llu)", static_cast<unsigned long long>(frameQueue.Dropped()),
        static_cast<unsigned long long>(imuQueue.Dropped()));

    std::_Exit(0);
}
