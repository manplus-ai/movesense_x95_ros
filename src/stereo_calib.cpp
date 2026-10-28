// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/stereo_calib.h"

#include <movesense/Simou3Camera.h>
#include <ros/ros.h>

namespace movesense_x95_ros {

StereoCalib::StereoCalib() {}

StereoCalib::~StereoCalib() {}

bool StereoCalib::Init(movesense::Simou3Camera* cam)
{
    ResetStereo();
    if (cam == nullptr) {
        ROS_ERROR("[StereoCalib] Init null argument");
        return false;
    }

    movesense::calib::StereoCalibration calibration;
    const int result = cam->getStereoCalibData(calibration);
    if (result != movesense::calib::kStereoCalibBytes) {
        ROS_WARN("[StereoCalib] getStereoCalibData failed (ret=%d)", result);
        return false;
    }

    if (!LoadStereoCalibration(calibration)) {
        ROS_WARN("[StereoCalib] stereo geometry is non-finite, degenerate or invalid");
        return false;
    }

    if (!HasImuExtrinsic()) {
        ROS_WARN("[StereoCalib] stereo IMU calibration is unavailable or invalid");
    }

    return true;
}

bool StereoCalib::InitColor(movesense::Simou3Camera* cam)
{
    ResetColor();
    if (cam == nullptr) {
        ROS_ERROR("[StereoCalib] InitColor null argument");
        return false;
    }

    movesense::calib::RgbCalibration calibration;
    const int result = cam->getRGBCalibData(calibration);
    if (result != movesense::calib::kRgbCalibBytes) {
        ROS_WARN("[StereoCalib] getRGBCalibData failed (ret=%d)", result);
        return false;
    }

    if (!LoadColorCalibration(calibration)) {
        ROS_WARN("[StereoCalib] color geometry is non-finite, degenerate or invalid");
        return false;
    }

    if (!HasColorImuExtrinsic()) {
        ROS_WARN("[StereoCalib] color IMU calibration is unavailable or invalid");
    }

    return true;
}

void StereoCalib::LogSummary() const
{
    if (IsValid()) {
        ROS_INFO("[StereoCalib] rectified intrinsics (1280 domain) fx=%.3f fy=%.3f cx=%.3f cy=%.3f baseline=%.4f mm", Fx(), Fy(), Cx(), Cy(),
            BaselineMm());
    } else {
        ROS_WARN("[StereoCalib] stereo geometry unavailable or invalid");
    }

    if (HasImuExtrinsic()) {
        const auto& imu = StereoImuCalibration();
        ROS_INFO("[StereoCalib] IMU to raw left t=[%.4f %.4f %.4f] mm ts=%.4f ms intrinsics=%s", imu.translationMm[0], imu.translationMm[1],
            imu.translationMm[2], imu.timeshiftMs, imu.hasIntrinsics ? "available" : "unavailable");
    }

    if (HasColor()) {
        ROS_INFO(
            "[StereoCalib] color rectified intrinsics (1280 domain) fx=%.3f fy=%.3f cx=%.3f cy=%.3f", ColorFx(), ColorFy(), ColorCx(), ColorCy());
    }

    if (HasColorImuExtrinsic()) {
        const auto& imu = ColorImuCalibration();
        ROS_INFO("[StereoCalib] IMU to raw color t=[%.4f %.4f %.4f] mm ts=%.4f ms intrinsics=%s", imu.translationMm[0], imu.translationMm[1],
            imu.translationMm[2], imu.timeshiftMs, imu.hasIntrinsics ? "available" : "unavailable");
    }
}

} // namespace movesense_x95_ros
