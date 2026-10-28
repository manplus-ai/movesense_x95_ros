// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace movesense_x95 {

struct ImageGeometry {
    uint32_t fullWidth = 0;
    uint32_t fullHeight = 0;
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t width = 0;
    uint32_t height = 0;
};

struct DetectionRectangle {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

bool ImageByteSize(uint32_t width, uint32_t height, size_t bytesPerPixel, size_t& size);

bool MakeImageGeometry(int fullWidth, int fullHeight, bool crop, int x1, int y1, int x2, int y2, ImageGeometry& result);

bool ImageGeometryMatches(const ImageGeometry& geometry, uint32_t width, uint32_t height);

bool ResolveImageGeometry(const ImageGeometry& configured, uint32_t width, uint32_t height, ImageGeometry& result);

bool AdjustCameraModel(const ImageGeometry& geometry, uint32_t width, uint32_t height, double* k, double* p);

bool ConvertDepthToMillimeters(const std::vector<uint8_t>& source, size_t pixelCount, double unitMm, std::vector<uint8_t>& output);

bool MapDetectionRectangle(const ImageGeometry& geometry, uint32_t coordWidth, uint32_t coordHeight, double x1, double y1,
    double x2, double y2, DetectionRectangle& result);

}
