// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include <array>

namespace movesense {
class Simou3Camera;
}
using movesense::Simou3Camera;

namespace movesense_x95_ros {

class StereoCalib {
public:
    StereoCalib();
    ~StereoCalib();

    bool Init(Simou3Camera* cam);
    bool InitColor(Simou3Camera* cam);
    void Shutdown();

    bool IsValid() const
    {
        return m_valid;
    }
    bool HasStereoBlob() const
    {
        return m_blobRead;
    }
    bool HasImuExtrinsic() const
    {
        return m_hasImu;
    }
    bool HasColor() const
    {
        return m_hasColor;
    }
    bool HasColorExtrinsic() const
    {
        return m_hasColorExtrinsic;
    }

    const std::array<double, 3>& ColorTregMm() const
    {
        return m_colorTregMm;
    }

    double ColorFx() const
    {
        return m_colorFx;
    }
    double ColorFy() const
    {
        return m_colorFy;
    }
    double ColorCx() const
    {
        return m_colorCx;
    }
    double ColorCy() const
    {
        return m_colorCy;
    }

    const std::array<double, 4>& LeftIntrinsics() const
    {
        return m_m1;
    }
    const std::array<double, 8>& LeftDistortion() const
    {
        return m_d1;
    }
    const std::array<double, 9>& LeftRectRotation() const
    {
        return m_ir1;
    }
    const std::array<double, 4>& RightIntrinsics() const
    {
        return m_m2;
    }
    const std::array<double, 8>& RightDistortion() const
    {
        return m_d2;
    }
    const std::array<double, 9>& RightRectRotation() const
    {
        return m_ir2;
    }

    double Fx() const
    {
        return m_fx;
    }
    double Fy() const
    {
        return m_fy;
    }
    double Cx() const
    {
        return m_cx;
    }
    double Cy() const
    {
        return m_cy;
    }
    double BaselineMm() const
    {
        return m_baselineMm;
    }

    const std::array<double, 9>& ImuRotation() const
    {
        return m_imuR;
    }
    const std::array<double, 3>& ImuTranslationMm() const
    {
        return m_imuT;
    }
    double ImuTimeshiftMs() const
    {
        return m_imuTsMs;
    }
    double GyroNoiseDensity() const
    {
        return m_gyroNd;
    }
    double GyroRandomWalk() const
    {
        return m_gyroRw;
    }
    double AccelNoiseDensity() const
    {
        return m_accNd;
    }
    double AccelRandomWalk() const
    {
        return m_accRw;
    }

    bool HasColorImuExtrinsic() const
    {
        return m_hasColorImu;
    }
    const std::array<double, 9>& ColorImuRotation() const
    {
        return m_colorImuR;
    }
    const std::array<double, 3>& ColorImuTranslationMm() const
    {
        return m_colorImuT;
    }
    double ColorImuTimeshiftMs() const
    {
        return m_colorImuTsMs;
    }
    double ColorGyroNoiseDensity() const
    {
        return m_colorGyroNd;
    }
    double ColorGyroRandomWalk() const
    {
        return m_colorGyroRw;
    }
    double ColorAccelNoiseDensity() const
    {
        return m_colorAccNd;
    }
    double ColorAccelRandomWalk() const
    {
        return m_colorAccRw;
    }

    double NegBaselineMm() const
    {
        return m_negBaselineMm;
    }

    const std::array<double, 4>& ColorIntrinsics() const
    {
        return m_colorM2;
    }
    const std::array<double, 8>& ColorDistortion() const
    {
        return m_colorD2;
    }
    const std::array<double, 9>& ColorInverseRectification() const
    {
        return m_colorIr2;
    }
    const std::array<double, 9>& ColorRotation() const
    {
        return m_colorR;
    }
    const std::array<double, 3>& ColorTranslationMm() const
    {
        return m_colorT;
    }

    void LogSummary() const;

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
    double m_imuTsMs = 0.0;
    double m_gyroNd = 0.0;
    double m_gyroRw = 0.0;
    double m_accNd = 0.0;
    double m_accRw = 0.0;

    bool m_hasColorImu = false;
    std::array<double, 9> m_colorImuR { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 3> m_colorImuT { { 0, 0, 0 } };
    double m_colorImuTsMs = 0.0;
    double m_colorGyroNd = 0.0;
    double m_colorGyroRw = 0.0;
    double m_colorAccNd = 0.0;
    double m_colorAccRw = 0.0;

    std::array<double, 4> m_colorM2 { { 0, 0, 0, 0 } };
    std::array<double, 8> m_colorD2 { { 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 9> m_colorIr2 { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 9> m_colorR { { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };
    std::array<double, 3> m_colorT { { 0, 0, 0 } };
};

} // namespace movesense_x95_ros
