// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95/seg_result.h"

#include <cstring>

namespace movesense_x95 {

bool SegValid(const void* buffer, size_t bytes)
{
    if (buffer == nullptr || bytes < sizeof(SegHeader)) {
        return false;
    }

    SegHeader header;
    std::memcpy(&header, buffer, sizeof(header));
    if (header.magic != kSegMagic || header.version != kSegVersion || header.coordW == 0 || header.coordH == 0 ||
        header.headerBytes < sizeof(SegHeader) || header.headerBytes > header.totalBytes || header.totalBytes != bytes ||
        header.detectionRecordBytes != sizeof(SegObject) || header.detectionsOffset < header.headerBytes ||
        header.detectionsOffset > header.totalBytes) {
        return false;
    }

    return header.detectionCount <= (header.totalBytes - header.detectionsOffset) / sizeof(SegObject);
}

}
