// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include <cstddef>
#include <cstdint>

namespace movesense_x95 {

enum : uint32_t { kSegMagic = 0x31524459u };
enum : uint16_t { kSegVersion = 1 };

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
};
#pragma pack(pop)

static_assert(sizeof(SegObject) == 24, "SegObject must be 24 bytes");
static_assert(sizeof(SegHeader) == 24, "SegHeader must be 24 bytes");

bool SegValid(const void* buffer, size_t bytes);

}
