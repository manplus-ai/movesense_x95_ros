// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include <cstdint>

namespace movesense_x95_ros {

enum : uint32_t { kSegMagic = 0x31524459u };

#pragma pack(push, 1)
struct SegObject {
    float x1, y1, x2, y2;
    float score;
    int32_t classId;
};

struct SegHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t headerBytes;
    uint32_t totalBytes;
    uint16_t coordW, coordH;
    uint16_t detectionCount;
    uint16_t detectionRecordBytes;
    uint32_t detectionsOffset;

    const SegObject* Detections() const
    {
        return reinterpret_cast<const SegObject*>(reinterpret_cast<const uint8_t*>(this) + detectionsOffset);
    }
};
#pragma pack(pop)

static_assert(sizeof(SegObject) == 24, "SegObject must be 24 bytes");
static_assert(sizeof(SegHeader) == 24, "SegHeader must be 24 bytes");

inline bool SegValid(const void* buf, int bytes)
{
    if (buf == nullptr || bytes < static_cast<int>(sizeof(SegHeader))) {
        return false;
    }

    const SegHeader* r = reinterpret_cast<const SegHeader*>(buf);

    if (r->magic != kSegMagic) {
        return false;
    }

    if (static_cast<int>(r->totalBytes) > bytes) {
        return false;
    }

    if (r->detectionsOffset + static_cast<uint32_t>(r->detectionCount) * sizeof(SegObject) > r->totalBytes) {
        return false;
    }

    return true;
}

} // namespace movesense_x95_ros
