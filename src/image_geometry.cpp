// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95/image_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace movesense_x95 {

bool ImageByteSize(uint32_t width, uint32_t height, size_t bytesPerPixel, size_t& size)
{
    if (width == 0 || height == 0 || bytesPerPixel == 0 ||
        width > std::numeric_limits<size_t>::max() / height / bytesPerPixel) {
        return false;
    }

    size = static_cast<size_t>(width) * height * bytesPerPixel;
    return true;
}

bool MakeImageGeometry(int fullWidth, int fullHeight, bool crop, int x1, int y1, int x2, int y2, ImageGeometry& result)
{
    result = ImageGeometry();
    if (fullWidth <= 0 || fullHeight <= 0) {
        return false;
    }
    if (crop && (x1 < 0 || y1 < 0 || x2 <= x1 || y2 <= y1 || x2 > fullWidth || y2 > fullHeight)) {
        return false;
    }

    result.fullWidth = static_cast<uint32_t>(fullWidth);
    result.fullHeight = static_cast<uint32_t>(fullHeight);
    result.x = crop ? static_cast<uint32_t>(x1) : 0;
    result.y = crop ? static_cast<uint32_t>(y1) : 0;
    result.width = crop ? static_cast<uint32_t>(x2 - x1) : result.fullWidth;
    result.height = crop ? static_cast<uint32_t>(y2 - y1) : result.fullHeight;
    return true;
}

bool ImageGeometryMatches(const ImageGeometry& geometry, uint32_t width, uint32_t height)
{
    return width > 0 && height > 0 && geometry.width == width && geometry.height == height;
}

bool ResolveImageGeometry(const ImageGeometry& configured, uint32_t width, uint32_t height, ImageGeometry& result)
{
    result = configured;
    if (configured.width == 0 || configured.height == 0) {
        return false;
    }
    if (width == 0 && height == 0) {
        return true;
    }
    if (ImageGeometryMatches(configured, width, height)) {
        return true;
    }
    if (width == 0 || height == 0 || configured.x != 0 || configured.y != 0 || configured.width != configured.fullWidth ||
        configured.height != configured.fullHeight) {
        return false;
    }

    result.fullWidth = width;
    result.fullHeight = height;
    result.width = width;
    result.height = height;
    return true;
}

bool AdjustCameraModel(const ImageGeometry& geometry, uint32_t width, uint32_t height, double* k, double* p)
{
    if (!ImageGeometryMatches(geometry, width, height) || k == nullptr || p == nullptr) {
        return false;
    }

    k[2] -= geometry.x;
    k[5] -= geometry.y;
    p[2] -= geometry.x;
    p[6] -= geometry.y;
    return true;
}

bool ConvertDepthToMillimeters(const std::vector<uint8_t>& source, size_t pixelCount, double unitMm, std::vector<uint8_t>& output)
{
    if (!std::isfinite(unitMm) || unitMm <= 0.0 || pixelCount > source.size() / 2) {
        return false;
    }

    const size_t bytes = pixelCount * 2;
    if (unitMm == 1.0) {
        output.assign(source.begin(), source.begin() + bytes);
        return true;
    }

    output.resize(bytes);
    for (size_t i = 0; i < pixelCount; ++i) {
        const uint16_t value = static_cast<uint16_t>(source[2 * i]) | static_cast<uint16_t>(source[2 * i + 1] << 8);
        const double millimeters = value * unitMm;
        const uint16_t converted = value == 0 || !std::isfinite(millimeters) || millimeters > 65535.0
            ? 0
            : static_cast<uint16_t>(std::floor(millimeters + 0.5));
        output[2 * i] = static_cast<uint8_t>(converted & 0xff);
        output[2 * i + 1] = static_cast<uint8_t>(converted >> 8);
    }
    return true;
}

bool MapDetectionRectangle(const ImageGeometry& geometry, uint32_t coordWidth, uint32_t coordHeight, double x1, double y1,
    double x2, double y2, DetectionRectangle& result)
{
    if (coordWidth == 0 || coordHeight == 0 || geometry.fullWidth == 0 || geometry.fullHeight == 0 || geometry.width == 0 ||
        geometry.height == 0 || !std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(x2) || !std::isfinite(y2) || x2 <= x1 || y2 <= y1) {
        return false;
    }

    const double scaleX = static_cast<double>(geometry.fullWidth) / coordWidth;
    const double scaleY = static_cast<double>(geometry.fullHeight) / coordHeight;
    result.x1 = std::max(0.0, std::min(static_cast<double>(geometry.width), x1 * scaleX - geometry.x));
    result.y1 = std::max(0.0, std::min(static_cast<double>(geometry.height), y1 * scaleY - geometry.y));
    result.x2 = std::max(0.0, std::min(static_cast<double>(geometry.width), x2 * scaleX - geometry.x));
    result.y2 = std::max(0.0, std::min(static_cast<double>(geometry.height), y2 * scaleY - geometry.y));
    return result.x2 > result.x1 && result.y2 > result.y1;
}

}
