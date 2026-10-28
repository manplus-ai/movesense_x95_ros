// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95/calibration_export.h"

#include <filesystem>
#include <initializer_list>
#include <opencv2/core.hpp>
#include <stdexcept>

namespace movesense_x95 {

template<typename T>
static cv::Mat Matrix(const T* values, int rows, int cols)
{
    cv::Mat result(rows, cols, CV_64F);
    for (int i = 0; i < rows * cols; ++i) {
        result.at<double>(i / cols, i % cols) = values[i];
    }
    return result;
}

static cv::Mat Intrinsics(const std::array<double, 4>& values)
{
    cv::Mat result = cv::Mat::eye(3, 3, CV_64F);
    result.at<double>(0, 0) = values[0];
    result.at<double>(1, 1) = values[1];
    result.at<double>(0, 2) = values[2];
    result.at<double>(1, 2) = values[3];
    return result;
}

static void WriteMatrix(const std::string& directory, const std::string& name, const cv::Mat& matrix)
{
    cv::FileStorage file(directory + "/" + name + ".xml", cv::FileStorage::WRITE);
    if (!file.isOpened()) {
        throw std::runtime_error("Cannot write " + directory + "/" + name + ".xml");
    }
    file << name << matrix;
}

static void InvalidateXmlFiles(const std::string& directory, std::initializer_list<const char*> names)
{
    for (const char* name : names) {
        const std::string path = directory + "/" + name + ".xml";
        if (std::filesystem::exists(path)) {
            cv::FileStorage file(path, cv::FileStorage::WRITE);
            if (!file.isOpened()) {
                throw std::runtime_error("Cannot invalidate " + path);
            }
            file << "valid" << 0;
        }
    }
}

static void InvalidateImu(const std::string& directory)
{
    InvalidateXmlFiles(directory, { "r_IMU", "t_IMU", "noise_IMU", "ts_IMU", "intrinsics_IMU" });
}

static void WriteImu(const std::string& directory, const movesense::calib::ImuCalibration& imu)
{
    WriteMatrix(directory, "r_IMU", Matrix(imu.rotation, 3, 3));
    WriteMatrix(directory, "t_IMU", Matrix(imu.translationMm, 1, 3));

    cv::FileStorage noise(directory + "/noise_IMU.xml", cv::FileStorage::WRITE);
    cv::FileStorage time(directory + "/ts_IMU.xml", cv::FileStorage::WRITE);
    cv::FileStorage intrinsics(directory + "/intrinsics_IMU.xml", cv::FileStorage::WRITE);
    if (!noise.isOpened() || !time.isOpened() || !intrinsics.isOpened()) {
        throw std::runtime_error("Cannot write IMU calibration in " + directory);
    }
    noise << "gyroscope_noise_density" << imu.gyroNoiseDensity;
    noise << "gyroscope_random_walk" << imu.gyroRandomWalk;
    noise << "accelerometer_noise_density" << imu.accelNoiseDensity;
    noise << "accelerometer_random_walk" << imu.accelRandomWalk;
    time << "timeshift_cam_imu" << imu.timeshiftMs;
    intrinsics << "model" << "scale-misalignment";
    intrinsics << "has_intrinsics" << static_cast<int>(imu.hasIntrinsics);
    if (imu.hasIntrinsics) {
        intrinsics << "accel_M" << Matrix(imu.accelM, 3, 3);
        intrinsics << "gyro_M" << Matrix(imu.gyroM, 3, 3);
        intrinsics << "gyro_A" << Matrix(imu.gyroA, 3, 3);
        intrinsics << "gyro_C" << Matrix(imu.gyroC, 3, 3);
    }
}

bool WriteCalibrationXml(const CalibrationData& calib, const std::string& directory, int cameraType, std::string& error)
{
    error.clear();
    if (directory.empty()) {
        return true;
    }

    try {
        std::filesystem::create_directories(directory);
        cv::FileStorage status(directory + "/calibration_status.xml", cv::FileStorage::WRITE);
        if (!status.isOpened()) {
            throw std::runtime_error("Cannot write calibration status in " + directory);
        }
        status << "sdk_version" << "0.5.2";
        status << "camera_type" << cameraType;
        status << "stereo_geometry_valid" << static_cast<int>(calib.IsValid());
        status << "rgb_geometry_valid" << static_cast<int>(calib.HasColorExtrinsic());
        status << "stereo_imu_valid" << static_cast<int>(calib.HasImuExtrinsic());
        status << "rgb_imu_valid" << static_cast<int>(calib.HasColorImuExtrinsic());
        status << "stereo_imu_intrinsics_valid" << static_cast<int>(calib.StereoImuCalibration().hasIntrinsics);
        status << "rgb_imu_intrinsics_valid" << static_cast<int>(calib.ColorImuCalibration().hasIntrinsics);
        status << "imu_extrinsic_reference" << "raw_camera";
        status << "translation_unit" << "mm";
        status << "timeshift_unit" << "ms";

        if (calib.IsValid()) {
            const std::string stereoDirectory = directory + "/xml";
            std::filesystem::create_directories(stereoDirectory);
            WriteMatrix(stereoDirectory, "M1", Intrinsics(calib.LeftIntrinsics()));
            WriteMatrix(stereoDirectory, "D1", Matrix(calib.LeftDistortion().data(), 8, 1));
            WriteMatrix(stereoDirectory, "M2", Intrinsics(calib.RightIntrinsics()));
            WriteMatrix(stereoDirectory, "D2", Matrix(calib.RightDistortion().data(), 8, 1));
            WriteMatrix(stereoDirectory, "IR_1", Matrix(calib.LeftRectRotation().data(), 3, 3));
            WriteMatrix(stereoDirectory, "IR_2", Matrix(calib.RightRectRotation().data(), 3, 3));

            cv::Mat projection = cv::Mat::zeros(3, 4, CV_64F);
            projection.at<double>(0, 0) = calib.Fx();
            projection.at<double>(1, 1) = calib.Fy();
            projection.at<double>(0, 2) = calib.Cx();
            projection.at<double>(1, 2) = calib.Cy();
            projection.at<double>(0, 3) = calib.NegBaselineMm() * calib.Fx();
            projection.at<double>(2, 2) = 1.0;
            WriteMatrix(stereoDirectory, "t_P2", projection);
            if (calib.HasImuExtrinsic()) {
                WriteImu(stereoDirectory, calib.StereoImuCalibration());
            } else {
                InvalidateImu(stereoDirectory);
            }
        } else {
            InvalidateXmlFiles(directory + "/xml", { "M1", "D1", "M2", "D2", "IR_1", "IR_2", "t_P2" });
            InvalidateImu(directory + "/xml");
        }

        if (cameraType != 1 && calib.HasColorExtrinsic()) {
            const std::string rgbDirectory = directory + "/xml_rgb";
            std::filesystem::create_directories(rgbDirectory);
            WriteMatrix(rgbDirectory, "M2", Intrinsics(calib.ColorIntrinsics()));
            WriteMatrix(rgbDirectory, "D2", Matrix(calib.ColorDistortion().data(), 8, 1));
            const std::array<double, 4> rectified = { calib.ColorFx(), calib.ColorFy(), calib.ColorCx(), calib.ColorCy() };
            WriteMatrix(rgbDirectory, "t_P2", Intrinsics(rectified));
            WriteMatrix(rgbDirectory, "iR_2", Matrix(calib.ColorInverseRectification().data(), 3, 3));
            WriteMatrix(rgbDirectory, "T_reg", Matrix(calib.ColorTregMm().data(), 3, 1));
            WriteMatrix(rgbDirectory, "R", Matrix(calib.ColorRotation().data(), 3, 3));
            WriteMatrix(rgbDirectory, "T", Matrix(calib.ColorTranslationMm().data(), 3, 1));
            if (calib.HasColorImuExtrinsic()) {
                WriteImu(rgbDirectory, calib.ColorImuCalibration());
            } else {
                InvalidateImu(rgbDirectory);
            }
        } else {
            InvalidateXmlFiles(directory + "/xml_rgb", { "M2", "D2", "t_P2", "iR_2", "T_reg", "R", "T" });
            InvalidateImu(directory + "/xml_rgb");
        }
        status << "export_complete" << 1;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
    return true;
}

} // namespace movesense_x95
