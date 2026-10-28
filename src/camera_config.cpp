// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95/camera_config.h"

namespace movesense_x95 {

const char* RoiStreamName(int stream)
{
    static const char* const names[kRoiStreamCount] = { "left_raw", "right_raw", "color_raw", "left_rect", "right_rect", "color_rect" };

    if (stream < 0 || stream >= kRoiStreamCount) {
        return "unknown";
    }

    return names[stream];
}

}
