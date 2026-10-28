// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include <cmath>
#include <movesense_x95/calibration_data.h>

namespace movesense_x95 {

template<typename T>
static bool AllFinite(const T* values, int count)
{
    for (int i = 0; i < count; ++i) {
        if (!std::isfinite(values[i])) {
            return false;
        }
    }

    return true;
}

template<typename T>
static bool IsRotation3x3(const T* matrix)
{
    const double kTolerance = 1e-3;
    if (!AllFinite(matrix, 9)) {
        return false;
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += static_cast<double>(matrix[k * 3 + row]) * matrix[k * 3 + col];
            }
            const double expected = row == col ? 1.0 : 0.0;
            if (std::fabs(sum - expected) > kTolerance) {
                return false;
            }
        }
    }

    const double det = static_cast<double>(matrix[0]) * (static_cast<double>(matrix[4]) * matrix[8] - static_cast<double>(matrix[5]) * matrix[7]) -
                       static_cast<double>(matrix[1]) * (static_cast<double>(matrix[3]) * matrix[8] - static_cast<double>(matrix[5]) * matrix[6]) +
                       static_cast<double>(matrix[2]) * (static_cast<double>(matrix[3]) * matrix[7] - static_cast<double>(matrix[4]) * matrix[6]);

    return std::isfinite(det) && std::fabs(det - 1.0) <= kTolerance;
}

static void Mul3x3(const double* left, const double* right, double* result)
{
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += left[row * 3 + k] * right[k * 3 + col];
            }
            result[row * 3 + col] = sum;
        }
    }
}

static void Mul3x3Vec(const double* matrix, const double* vector, double* result)
{
    for (int row = 0; row < 3; ++row) {
        result[row] = matrix[row * 3] * vector[0] + matrix[row * 3 + 1] * vector[1] + matrix[row * 3 + 2] * vector[2];
    }
}

template<typename T, std::size_t N>
static void CopyValues(const T* values, std::array<double, N>& result)
{
    for (std::size_t i = 0; i < N; ++i) {
        result[i] = values[i];
    }
}

static bool ValidIntrinsics(const movesense::calib::Intrinsics& intrinsics)
{
    return std::isfinite(intrinsics.fx) && std::isfinite(intrinsics.fy) && std::isfinite(intrinsics.cx) && std::isfinite(intrinsics.cy) &&
           intrinsics.fx > 1.0f && intrinsics.fy > 1.0f;
}

static void CopyIntrinsics(const movesense::calib::Intrinsics& intrinsics, std::array<double, 4>& result)
{
    result = { { intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy } };
}

static bool RecoverRectification(
    const float* inverseRectify, const movesense::calib::Intrinsics& intrinsics, std::array<double, 9>& rawFromRect, std::array<double, 9>& rawToRect)
{
    if (!ValidIntrinsics(intrinsics) || !AllFinite(inverseRectify, 9)) {
        return false;
    }

    std::array<double, 9> inverse;
    CopyValues(inverseRectify, inverse);
    const double projection[9] = { intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy, intrinsics.cy, 0.0, 0.0, 1.0 };
    Mul3x3(inverse.data(), projection, rawFromRect.data());
    if (!IsRotation3x3(rawFromRect.data())) {
        return false;
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rawToRect[row * 3 + col] = rawFromRect[col * 3 + row];
        }
    }

    return true;
}

static bool ValidScaleMatrix(const float* matrix, double minimum, double maximum)
{
    if (!AllFinite(matrix, 9)) {
        return false;
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const double value = matrix[row * 3 + col];
            if ((row == col && (value < minimum || value > maximum)) || (col > row && value != 0.0) || (row > col && std::fabs(value) > 0.05)) {
                return false;
            }
        }
    }

    return true;
}

static bool ValidImuCalibration(const movesense::calib::ImuCalibration& imu)
{
    const float scalars[5] = { imu.gyroNoiseDensity, imu.gyroRandomWalk, imu.accelNoiseDensity, imu.accelRandomWalk, imu.timeshiftMs };
    if (!AllFinite(scalars, 5) || imu.gyroNoiseDensity <= 0.0f || imu.gyroRandomWalk <= 0.0f || imu.accelNoiseDensity <= 0.0f ||
        imu.accelRandomWalk <= 0.0f || !IsRotation3x3(imu.rotation) || !AllFinite(imu.translationMm, 3)) {
        return false;
    }

    if (!imu.hasIntrinsics) {
        return true;
    }

    if (!ValidScaleMatrix(imu.accelM, 0.95, 1.05) || !ValidScaleMatrix(imu.gyroM, 0.90, 1.10) || !AllFinite(imu.gyroA, 9) ||
        !IsRotation3x3(imu.gyroC)) {
        return false;
    }

    for (int i = 0; i < 9; ++i) {
        if (std::fabs(imu.gyroA[i]) > 0.02) {
            return false;
        }
    }

    const double cosine = (static_cast<double>(imu.gyroC[0]) + imu.gyroC[4] + imu.gyroC[8] - 1.0) / 2.0;
    return cosine >= std::cos(2.0 * 3.14159265358979323846 / 180.0);
}

CalibrationData::CalibrationData() {}

CalibrationData::~CalibrationData() {}

void CalibrationData::ResetStereo()
{
    m_valid = false;
    m_blobRead = false;
    m_hasImu = false;
    m_fx = 0.0;
    m_fy = 0.0;
    m_cx = 0.0;
    m_cy = 0.0;
    m_baselineMm = 0.0;
    m_negBaselineMm = 0.0;
    m_m1.fill(0.0);
    m_m2.fill(0.0);
    m_d1.fill(0.0);
    m_d2.fill(0.0);
    m_ir1.fill(0.0);
    m_ir2.fill(0.0);
    m_imuR.fill(0.0);
    m_imuT.fill(0.0);
    m_leftRawToRect.fill(0.0);
    m_stereoImu = movesense::calib::ImuCalibration();
}

void CalibrationData::ResetColor()
{
    m_hasColor = false;
    m_hasColorExtrinsic = false;
    m_hasColorImu = false;
    m_colorFx = 0.0;
    m_colorFy = 0.0;
    m_colorCx = 0.0;
    m_colorCy = 0.0;
    m_colorM2.fill(0.0);
    m_colorD2.fill(0.0);
    m_colorIr2.fill(0.0);
    m_colorR.fill(0.0);
    m_colorT.fill(0.0);
    m_colorTregMm.fill(0.0);
    m_colorImuR.fill(0.0);
    m_colorImuT.fill(0.0);
    m_colorRawToRect.fill(0.0);
    m_colorImu = movesense::calib::ImuCalibration();
}

bool CalibrationData::LoadStereoCalibration(const movesense::calib::StereoCalibration& calibration)
{
    ResetStereo();
    std::array<double, 9> leftRawFromRect;
    std::array<double, 9> rightRawFromRect;
    std::array<double, 9> rightRawToRect;
    if (!ValidIntrinsics(calibration.left.intrinsics) || !ValidIntrinsics(calibration.right.intrinsics) ||
        !AllFinite(calibration.left.distortion, 8) || !AllFinite(calibration.right.distortion, 8) || !std::isfinite(calibration.negativeBaselineMm) ||
        std::fabs(calibration.negativeBaselineMm) <= 1e-6 ||
        !RecoverRectification(calibration.left.inverseRectify, calibration.rectified, leftRawFromRect, m_leftRawToRect) ||
        !RecoverRectification(calibration.right.inverseRectify, calibration.rectified, rightRawFromRect, rightRawToRect)) {
        return false;
    }

    m_fx = calibration.rectified.fx;
    m_fy = calibration.rectified.fy;
    m_cx = calibration.rectified.cx;
    m_cy = calibration.rectified.cy;
    m_negBaselineMm = calibration.negativeBaselineMm;
    m_baselineMm = std::fabs(m_negBaselineMm);
    CopyIntrinsics(calibration.left.intrinsics, m_m1);
    CopyIntrinsics(calibration.right.intrinsics, m_m2);
    CopyValues(calibration.left.distortion, m_d1);
    CopyValues(calibration.right.distortion, m_d2);
    CopyValues(calibration.left.inverseRectify, m_ir1);
    CopyValues(calibration.right.inverseRectify, m_ir2);

    m_hasImu = ValidImuCalibration(calibration.imu);
    if (m_hasImu) {
        m_stereoImu = calibration.imu;
        CopyValues(calibration.imu.rotation, m_imuR);
        CopyValues(calibration.imu.translationMm, m_imuT);
    }

    m_valid = true;
    m_blobRead = true;
    return true;
}

bool CalibrationData::LoadColorCalibration(const movesense::calib::RgbCalibration& calibration)
{
    ResetColor();
    if (!ValidIntrinsics(calibration.intrinsics) || !AllFinite(calibration.distortion, 8) || !AllFinite(calibration.registeredTranslation, 3) ||
        !RecoverRectification(calibration.inverseRectify, calibration.rectified, m_colorR, m_colorRawToRect)) {
        return false;
    }

    m_colorFx = calibration.rectified.fx;
    m_colorFy = calibration.rectified.fy;
    m_colorCx = calibration.rectified.cx;
    m_colorCy = calibration.rectified.cy;
    CopyIntrinsics(calibration.intrinsics, m_colorM2);
    CopyValues(calibration.distortion, m_colorD2);
    CopyValues(calibration.inverseRectify, m_colorIr2);
    CopyValues(calibration.registeredTranslation, m_colorTregMm);
    Mul3x3Vec(m_colorR.data(), m_colorTregMm.data(), m_colorT.data());

    m_hasColorImu = ValidImuCalibration(calibration.imu);
    if (m_hasColorImu) {
        m_colorImu = calibration.imu;
        CopyValues(calibration.imu.rotation, m_colorImuR);
        CopyValues(calibration.imu.translationMm, m_colorImuT);
    }

    m_hasColor = true;
    m_hasColorExtrinsic = true;
    return true;
}

bool CalibrationData::SelectImuCalibration(int cameraType, bool& color) const
{
    const bool stereoAvailable = m_valid && m_hasImu;
    const bool colorAvailable = m_hasColorExtrinsic && m_hasColorImu;
    color = cameraType == 0;
    if ((color && colorAvailable) || (!color && stereoAvailable)) {
        return true;
    }
    if (colorAvailable || stereoAvailable) {
        color = colorAvailable;
        return true;
    }

    color = false;
    return false;
}

bool CalibrationData::GetRectifiedImuTransform(bool color, std::array<double, 9>& rotation, std::array<double, 3>& translationMm) const
{
    rotation.fill(0.0);
    translationMm.fill(0.0);
    if ((color && (!m_hasColorExtrinsic || !m_hasColorImu)) || (!color && (!m_valid || !m_hasImu))) {
        return false;
    }

    const std::array<double, 9>& rawToRect = color ? m_colorRawToRect : m_leftRawToRect;
    const std::array<double, 9>& rawRotation = color ? m_colorImuR : m_imuR;
    const std::array<double, 3>& rawTranslation = color ? m_colorImuT : m_imuT;
    Mul3x3(rawToRect.data(), rawRotation.data(), rotation.data());
    Mul3x3Vec(rawToRect.data(), rawTranslation.data(), translationMm.data());

    return IsRotation3x3(rotation.data()) && AllFinite(translationMm.data(), 3);
}

void CalibrationData::Shutdown()
{
    ResetStereo();
    ResetColor();
}

const movesense::calib::ImuCalibration& CalibrationData::StereoImuCalibration() const
{
    return m_stereoImu;
}

const movesense::calib::ImuCalibration& CalibrationData::ColorImuCalibration() const
{
    return m_colorImu;
}

bool CalibrationData::IsValid() const
{
    return m_valid;
}

bool CalibrationData::HasStereoBlob() const
{
    return m_blobRead;
}

bool CalibrationData::HasImuExtrinsic() const
{
    return m_hasImu;
}

bool CalibrationData::HasColor() const
{
    return m_hasColor;
}

bool CalibrationData::HasColorExtrinsic() const
{
    return m_hasColorExtrinsic;
}

const std::array<double, 3>& CalibrationData::ColorTregMm() const
{
    return m_colorTregMm;
}

double CalibrationData::ColorFx() const
{
    return m_colorFx;
}

double CalibrationData::ColorFy() const
{
    return m_colorFy;
}

double CalibrationData::ColorCx() const
{
    return m_colorCx;
}

double CalibrationData::ColorCy() const
{
    return m_colorCy;
}

const std::array<double, 4>& CalibrationData::LeftIntrinsics() const
{
    return m_m1;
}

const std::array<double, 8>& CalibrationData::LeftDistortion() const
{
    return m_d1;
}

const std::array<double, 9>& CalibrationData::LeftRectRotation() const
{
    return m_ir1;
}

const std::array<double, 4>& CalibrationData::RightIntrinsics() const
{
    return m_m2;
}

const std::array<double, 8>& CalibrationData::RightDistortion() const
{
    return m_d2;
}

const std::array<double, 9>& CalibrationData::RightRectRotation() const
{
    return m_ir2;
}

double CalibrationData::Fx() const
{
    return m_fx;
}

double CalibrationData::Fy() const
{
    return m_fy;
}

double CalibrationData::Cx() const
{
    return m_cx;
}

double CalibrationData::Cy() const
{
    return m_cy;
}

double CalibrationData::BaselineMm() const
{
    return m_baselineMm;
}

const std::array<double, 9>& CalibrationData::ImuRotation() const
{
    return m_imuR;
}

const std::array<double, 3>& CalibrationData::ImuTranslationMm() const
{
    return m_imuT;
}

double CalibrationData::ImuTimeshiftMs() const
{
    return m_stereoImu.timeshiftMs;
}

double CalibrationData::GyroNoiseDensity() const
{
    return m_stereoImu.gyroNoiseDensity;
}

double CalibrationData::GyroRandomWalk() const
{
    return m_stereoImu.gyroRandomWalk;
}

double CalibrationData::AccelNoiseDensity() const
{
    return m_stereoImu.accelNoiseDensity;
}

double CalibrationData::AccelRandomWalk() const
{
    return m_stereoImu.accelRandomWalk;
}

bool CalibrationData::HasColorImuExtrinsic() const
{
    return m_hasColorImu;
}

const std::array<double, 9>& CalibrationData::ColorImuRotation() const
{
    return m_colorImuR;
}

const std::array<double, 3>& CalibrationData::ColorImuTranslationMm() const
{
    return m_colorImuT;
}

double CalibrationData::ColorImuTimeshiftMs() const
{
    return m_colorImu.timeshiftMs;
}

double CalibrationData::ColorGyroNoiseDensity() const
{
    return m_colorImu.gyroNoiseDensity;
}

double CalibrationData::ColorGyroRandomWalk() const
{
    return m_colorImu.gyroRandomWalk;
}

double CalibrationData::ColorAccelNoiseDensity() const
{
    return m_colorImu.accelNoiseDensity;
}

double CalibrationData::ColorAccelRandomWalk() const
{
    return m_colorImu.accelRandomWalk;
}

double CalibrationData::NegBaselineMm() const
{
    return m_negBaselineMm;
}

const std::array<double, 4>& CalibrationData::ColorIntrinsics() const
{
    return m_colorM2;
}

const std::array<double, 8>& CalibrationData::ColorDistortion() const
{
    return m_colorD2;
}

const std::array<double, 9>& CalibrationData::ColorInverseRectification() const
{
    return m_colorIr2;
}

const std::array<double, 9>& CalibrationData::ColorRotation() const
{
    return m_colorR;
}

const std::array<double, 3>& CalibrationData::ColorTranslationMm() const
{
    return m_colorT;
}

} // namespace movesense_x95
