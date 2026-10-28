// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include <array>
#include <movesense/Simou3CalibLayout.h>

namespace movesense_x95 {

class CalibrationData {
public:
    CalibrationData();
    ~CalibrationData();

    bool LoadStereoCalibration(const movesense::calib::StereoCalibration& calibration);
    bool LoadColorCalibration(const movesense::calib::RgbCalibration& calibration);
    bool SelectImuCalibration(int cameraType, bool& color) const;
    bool GetRectifiedImuTransform(bool color, std::array<double, 9>& rotation, std::array<double, 3>& translationMm) const;
    void Shutdown();

    const movesense::calib::ImuCalibration& StereoImuCalibration() const;
    const movesense::calib::ImuCalibration& ColorImuCalibration() const;

    bool IsValid() const;
    bool HasStereoBlob() const;
    bool HasImuExtrinsic() const;
    bool HasColor() const;
    bool HasColorExtrinsic() const;

    const std::array<double, 3>& ColorTregMm() const;

    double ColorFx() const;
    double ColorFy() const;
    double ColorCx() const;
    double ColorCy() const;

    const std::array<double, 4>& LeftIntrinsics() const;
    const std::array<double, 8>& LeftDistortion() const;
    const std::array<double, 9>& LeftRectRotation() const;
    const std::array<double, 4>& RightIntrinsics() const;
    const std::array<double, 8>& RightDistortion() const;
    const std::array<double, 9>& RightRectRotation() const;

    double Fx() const;
    double Fy() const;
    double Cx() const;
    double Cy() const;
    double BaselineMm() const;

    const std::array<double, 9>& ImuRotation() const;
    const std::array<double, 3>& ImuTranslationMm() const;
    double ImuTimeshiftMs() const;
    double GyroNoiseDensity() const;
    double GyroRandomWalk() const;
    double AccelNoiseDensity() const;
    double AccelRandomWalk() const;

    bool HasColorImuExtrinsic() const;
    const std::array<double, 9>& ColorImuRotation() const;
    const std::array<double, 3>& ColorImuTranslationMm() const;
    double ColorImuTimeshiftMs() const;
    double ColorGyroNoiseDensity() const;
    double ColorGyroRandomWalk() const;
    double ColorAccelNoiseDensity() const;
    double ColorAccelRandomWalk() const;

    double NegBaselineMm() const;

    const std::array<double, 4>& ColorIntrinsics() const;
    const std::array<double, 8>& ColorDistortion() const;
    const std::array<double, 9>& ColorInverseRectification() const;
    const std::array<double, 9>& ColorRotation() const;
    const std::array<double, 3>& ColorTranslationMm() const;

protected:
    void ResetStereo();
    void ResetColor();

private:
    bool m_valid = false;
    bool m_blobRead = false;
    bool m_hasImu = false;
    bool m_hasColor = false;
    bool m_hasColorExtrinsic = false;

    std::array<double, 3> m_colorTregMm { { 0, 0, 0 } };

    double m_colorFx = 0.0;
    double m_colorFy = 0.0;
    double m_colorCx = 0.0;
    double m_colorCy = 0.0;

    std::array<double, 4> m_m1 { { 0, 0, 0, 0 } };
    std::array<double, 8> m_d1 { { 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 9> m_ir1 { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

    std::array<double, 4> m_m2 { { 0, 0, 0, 0 } };
    std::array<double, 8> m_d2 { { 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 9> m_ir2 { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

    double m_fx = 0.0;
    double m_fy = 0.0;
    double m_cx = 0.0;
    double m_cy = 0.0;
    double m_baselineMm = 0.0;
    double m_negBaselineMm = 0.0;

    std::array<double, 9> m_imuR { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 3> m_imuT { { 0, 0, 0 } };
    movesense::calib::ImuCalibration m_stereoImu;
    std::array<double, 9> m_leftRawToRect { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

    bool m_hasColorImu = false;
    std::array<double, 9> m_colorImuR { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 3> m_colorImuT { { 0, 0, 0 } };
    movesense::calib::ImuCalibration m_colorImu;
    std::array<double, 9> m_colorRawToRect { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

    std::array<double, 4> m_colorM2 { { 0, 0, 0, 0 } };
    std::array<double, 8> m_colorD2 { { 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 9> m_colorIr2 { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 9> m_colorR { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 3> m_colorT { { 0, 0, 0 } };
};

}

