// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/stereo_calib.h"

#include <cmath>
#include <cstring>
#include <movesense/Simou3Camera.h>
#include <ros/ros.h>
#include <vector>

using namespace movesense;

namespace movesense_x95_ros {

static bool IsIdentity3x3(const double* m, double tol)
{
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const double expected = (row == col) ? 1.0 : 0.0;
            if (std::fabs(m[row * 3 + col] - expected) > tol) {
                return false;
            }
        }
    }
    return true;
}

static bool IsRotation3x3(const double* m, double tol)
{
    double mtm[9] = { 0 };
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += m[k * 3 + row] * m[k * 3 + col];
            }
            mtm[row * 3 + col] = sum;
        }
    }

    if (!IsIdentity3x3(mtm, tol)) {
        return false;
    }

    const double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);

    return std::fabs(det - 1.0) <= tol;
}

static void Mul3x3(const double* a, const double* b, double* out)
{
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += a[row * 3 + k] * b[k * 3 + col];
            }
            out[row * 3 + col] = sum;
        }
    }
}

static void Mul3x3Vec(const double* a, const double* v, double* out)
{
    for (int row = 0; row < 3; ++row) {
        out[row] = a[row * 3 + 0] * v[0] + a[row * 3 + 1] * v[1] + a[row * 3 + 2] * v[2];
    }
}

static bool AllFinite(const double* v, int n)
{
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(v[i])) {
            return false;
        }
    }
    return true;
}

StereoCalib::StereoCalib() {}

StereoCalib::~StereoCalib() {}

bool StereoCalib::Init(Simou3Camera* cam)
{
    if (cam == nullptr) {
        ROS_ERROR("[StereoCalib] Init null argument");
        return false;
    }

    const int N = calib::kStereoCalibFloats;
    std::vector<unsigned char> buf(static_cast<size_t>(N) * 4, 0);

    int ret = cam->getStereoCalibData(buf.data(), N * 4);
    if (ret <= 0) {
        ROS_WARN("[StereoCalib] getStereoCalibData failed (ret=%d); camera may be uncalibrated", ret);
        m_valid = false;
        return false;
    }

    m_blobRead = true;

    float d[N];
    std::memcpy(d, buf.data(), sizeof(d));

    m_fx = d[calib::kRectifiedFx];
    m_fy = d[calib::kRectifiedFy];
    m_cx = d[calib::kRectifiedCx];
    m_cy = d[calib::kRectifiedCy];

    const float negBaselineMm = d[calib::kStereoNegativeBaselineMm];
    m_baselineMm = (negBaselineMm < 0.0f) ? -static_cast<double>(negBaselineMm) : static_cast<double>(negBaselineMm);
    m_negBaselineMm = static_cast<double>(negBaselineMm);

    for (int i = 0; i < 4; ++i) {
        m_m1[i] = d[calib::kStereoLeftIntrinsics + i];
    }

    for (int i = 0; i < 9; ++i) {
        m_ir1[i] = d[calib::kStereoLeftInverseRectifyRowMajor + i];
    }

    for (int i = 0; i < 8; ++i) {
        m_d1[i] = d[calib::kStereoLeftDistortion + i];
    }

    for (int i = 0; i < 4; ++i) {
        m_m2[i] = d[calib::kStereoRightIntrinsics + i];
    }

    for (int i = 0; i < 9; ++i) {
        m_ir2[i] = d[calib::kStereoRightInverseRectifyRowMajor + i];
    }

    for (int i = 0; i < 8; ++i) {
        m_d2[i] = d[calib::kStereoRightDistortion + i];
    }

    const float* seg = d + calib::kStereoImuSegment;
    bool anyNonZero = false;
    bool allFinite = true;
    for (int i = 0; i < calib::kImuSegmentFloats; ++i) {
        if (!std::isfinite(seg[i]) || std::fabs(seg[i]) > 1e6f) {
            allFinite = false;
            break;
        }
        if (seg[i] != 0.0f) {
            anyNonZero = true;
        }
    }

    if (!allFinite) {
        ROS_WARN("[StereoCalib] IMU extrinsic segment is out of range or non-finite; treated as uncalibrated");
    }

    m_hasImu = anyNonZero && allFinite;

    if (m_hasImu) {
        m_gyroNd = seg[calib::kImuGyroNoiseDensity];
        m_gyroRw = seg[calib::kImuGyroRandomWalk];
        m_accNd = seg[calib::kImuAccelNoiseDensity];
        m_accRw = seg[calib::kImuAccelRandomWalk];
        m_imuTsMs = seg[calib::kImuTimeshiftMs];

        for (int i = 0; i < 9; ++i) {
            m_imuR[i] = seg[calib::kImuRotationRowMajor + i];
        }
        for (int i = 0; i < 3; ++i) {
            m_imuT[i] = seg[calib::kImuTranslationMm + i];
        }
    }

    if (m_fx <= 1.0 || m_fy <= 1.0) {
        ROS_WARN("[StereoCalib] invalid rectified intrinsics (fx=%.3f fy=%.3f); stereo may be uncalibrated", m_fx, m_fy);
        m_valid = false;
        return false;
    }

    m_valid = true;
    return true;
}

bool StereoCalib::InitColor(Simou3Camera* cam)
{
    if (cam == nullptr) {
        ROS_ERROR("[StereoCalib] InitColor null argument");
        return false;
    }

    const int N = calib::kRgbCalibFloats;
    std::vector<unsigned char> buf(static_cast<size_t>(N) * 4, 0);

    int ret = cam->getRGBCalibData(buf.data(), N * 4);
    if (ret <= 0) {
        ROS_WARN("[StereoCalib] getRGBCalibData failed (ret=%d); color may be uncalibrated", ret);
        m_hasColor = false;
        return false;
    }

    float d[N];
    std::memcpy(d, buf.data(), sizeof(d));

    m_colorFx = d[calib::kRectifiedFx];
    m_colorFy = d[calib::kRectifiedFy];
    m_colorCx = d[calib::kRectifiedCx];
    m_colorCy = d[calib::kRectifiedCy];

    if (m_colorFx <= 1.0 || m_colorFy <= 1.0) {
        ROS_WARN("[StereoCalib] invalid color intrinsics (fx=%.3f fy=%.3f); color uncalibrated", m_colorFx, m_colorFy);
        m_hasColor = false;
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        m_colorM2[i] = d[calib::kRgbIntrinsics + i];
    }

    for (int i = 0; i < 8; ++i) {
        m_colorD2[i] = d[calib::kRgbDistortion + i];
    }

    double ir[9];
    for (int i = 0; i < 9; ++i) {
        ir[i] = d[calib::kRgbInverseRectifyRowMajor + i];
    }

    double rp[9];
    for (int i = 0; i < 9; ++i) {
        rp[i] = d[calib::kRgbRotationRowMajor + i];
    }

    double tp[3];
    for (int i = 0; i < 3; ++i) {
        tp[i] = d[calib::kRgbTranslationMm + i];
    }

    const double proj[9] = { m_colorFx, 0.0, m_colorCx, 0.0, m_colorFy, m_colorCy, 0.0, 0.0, 1.0 };
    const double projInv[9]
        = { 1.0 / m_colorFx, 0.0, -m_colorCx / m_colorFx, 0.0, 1.0 / m_colorFy, -m_colorCy / m_colorFy, 0.0, 0.0, 1.0 };

    const bool payloadUsable = AllFinite(m_colorM2.data(), 4) && AllFinite(m_colorD2.data(), 8) && AllFinite(ir, 9) && AllFinite(tp, 3)
        && std::fabs(m_colorM2[0] * m_colorM2[1]) > 1e-12;

    if (!payloadUsable) {
        ROS_WARN("[StereoCalib] color calibration payload is non-finite or degenerate; color extrinsic unusable");
        m_hasColorExtrinsic = false;
    } else if (IsIdentity3x3(rp, 1e-5)) {
        for (int i = 0; i < 3; ++i) {
            m_colorTregMm[i] = tp[i];
        }
        for (int i = 0; i < 9; ++i) {
            m_colorIr2[i] = ir[i];
        }
        Mul3x3(m_colorIr2.data(), proj, m_colorR.data());
        Mul3x3Vec(m_colorR.data(), m_colorTregMm.data(), m_colorT.data());
        m_hasColorExtrinsic = true;
    } else if (IsRotation3x3(rp, 1e-3)) {
        for (int i = 0; i < 9; ++i) {
            m_colorR[i] = rp[i];
        }
        for (int i = 0; i < 3; ++i) {
            m_colorT[i] = tp[i];
        }
        Mul3x3(m_colorR.data(), projInv, m_colorIr2.data());
        for (int row = 0; row < 3; ++row) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += m_colorR[k * 3 + row] * m_colorT[k];
            }
            m_colorTregMm[row] = sum;
        }
        m_hasColorExtrinsic = true;
    } else {
        ROS_WARN("[StereoCalib] color extrinsic rotation slot is neither identity nor a valid rotation; color extrinsic unusable");
        m_hasColorExtrinsic = false;
    }

    for (int i = 0; i < 3 && m_hasColorExtrinsic; ++i) {
        if (!std::isfinite(m_colorTregMm[i])) {
            ROS_WARN("[StereoCalib] color extrinsic translation is not finite; color extrinsic unusable");
            m_hasColorExtrinsic = false;
        }
    }

    const float* cseg = d + calib::kRgbImuSegment;
    bool cAnyNonZero = false;
    bool cAllFinite = true;
    for (int i = 0; i < calib::kImuSegmentFloats; ++i) {
        if (!std::isfinite(cseg[i]) || std::fabs(cseg[i]) > 1e6f) {
            cAllFinite = false;
            break;
        }
        if (cseg[i] != 0.0f) {
            cAnyNonZero = true;
        }
    }

    if (!cAllFinite) {
        ROS_WARN("[StereoCalib] color IMU extrinsic segment is out of range or non-finite; treated as uncalibrated");
    }

    m_hasColorImu = cAnyNonZero && cAllFinite;

    if (m_hasColorImu) {
        m_colorGyroNd = cseg[calib::kImuGyroNoiseDensity];
        m_colorGyroRw = cseg[calib::kImuGyroRandomWalk];
        m_colorAccNd = cseg[calib::kImuAccelNoiseDensity];
        m_colorAccRw = cseg[calib::kImuAccelRandomWalk];
        m_colorImuTsMs = cseg[calib::kImuTimeshiftMs];

        for (int i = 0; i < 9; ++i) {
            m_colorImuR[i] = cseg[calib::kImuRotationRowMajor + i];
        }
        for (int i = 0; i < 3; ++i) {
            m_colorImuT[i] = cseg[calib::kImuTranslationMm + i];
        }
    }

    m_hasColor = true;
    ROS_INFO("[StereoCalib] color rectified intrinsics (1280 domain) fx=%.3f fy=%.3f cx=%.3f cy=%.3f", m_colorFx, m_colorFy, m_colorCx, m_colorCy);
    return true;
}

void StereoCalib::Shutdown() {}

void StereoCalib::LogSummary() const
{
    if (!m_valid) {
        ROS_WARN("[StereoCalib] extrinsics invalid (not read or uncalibrated)");
        return;
    }

    ROS_INFO("[StereoCalib] rectified intrinsics (1280 domain, shared) fx=%.3f fy=%.3f cx=%.3f cy=%.3f | baseline=%.4f mm", m_fx, m_fy, m_cx, m_cy,
        m_baselineMm);
    ROS_INFO("[StereoCalib] left  M1 fx=%.3f fy=%.3f cx=%.3f cy=%.3f | D1[0..3]=%.4f %.4f %.4f %.4f", m_m1[0], m_m1[1], m_m1[2], m_m1[3], m_d1[0],
        m_d1[1], m_d1[2], m_d1[3]);
    ROS_INFO("[StereoCalib] right M2 fx=%.3f fy=%.3f cx=%.3f cy=%.3f | D2[0..3]=%.4f %.4f %.4f %.4f", m_m2[0], m_m2[1], m_m2[2], m_m2[3], m_d2[0],
        m_d2[1], m_d2[2], m_d2[3]);
    ROS_INFO("[StereoCalib] left  rect rotation IR_1=[%.5f %.5f %.5f; %.5f %.5f %.5f; %.5f %.5f %.5f]", m_ir1[0], m_ir1[1], m_ir1[2], m_ir1[3],
        m_ir1[4], m_ir1[5], m_ir1[6], m_ir1[7], m_ir1[8]);
    ROS_INFO("[StereoCalib] right rect rotation IR_2=[%.5f %.5f %.5f; %.5f %.5f %.5f; %.5f %.5f %.5f]", m_ir2[0], m_ir2[1], m_ir2[2], m_ir2[3],
        m_ir2[4], m_ir2[5], m_ir2[6], m_ir2[7], m_ir2[8]);

    if (m_hasImu) {
        ROS_INFO("[StereoCalib] IMU<->left R=[%.5f %.5f %.5f; %.5f %.5f %.5f; %.5f %.5f %.5f]", m_imuR[0], m_imuR[1], m_imuR[2], m_imuR[3], m_imuR[4],
            m_imuR[5], m_imuR[6], m_imuR[7], m_imuR[8]);
        ROS_INFO("[StereoCalib] IMU<->left t=[%.4f %.4f %.4f] mm ts=%.4f ms | noise gNd=%.3e gRw=%.3e aNd=%.3e aRw=%.3e", m_imuT[0], m_imuT[1],
            m_imuT[2], m_imuTsMs, m_gyroNd, m_gyroRw, m_accNd, m_accRw);
    } else {
        ROS_INFO("[StereoCalib] IMU<->left extrinsic unusable: segment is all zero or non-finite (no IMU calib uploaded)");
    }

    if (!m_hasColor) {
        return;
    }

    if (m_hasColorImu) {
        ROS_INFO("[StereoCalib] IMU<->color R=[%.5f %.5f %.5f; %.5f %.5f %.5f; %.5f %.5f %.5f]", m_colorImuR[0], m_colorImuR[1], m_colorImuR[2],
            m_colorImuR[3], m_colorImuR[4], m_colorImuR[5], m_colorImuR[6], m_colorImuR[7], m_colorImuR[8]);
        ROS_INFO("[StereoCalib] IMU<->color t=[%.4f %.4f %.4f] mm ts=%.4f ms | noise gNd=%.3e gRw=%.3e aNd=%.3e aRw=%.3e", m_colorImuT[0],
            m_colorImuT[1], m_colorImuT[2], m_colorImuTsMs, m_colorGyroNd, m_colorGyroRw, m_colorAccNd, m_colorAccRw);
    } else {
        ROS_INFO("[StereoCalib] IMU<->color extrinsic unusable: segment is all zero or non-finite (no IMU calib uploaded)");
    }
}

} // namespace movesense_x95_ros
