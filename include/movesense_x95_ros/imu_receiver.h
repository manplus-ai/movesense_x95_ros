// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95/data_receiver.h"

namespace movesense_x95_ros {

using movesense_x95::ImuQueue;

class ImuReceiver : public movesense_x95::RawImuReceiver {
public:
    ImuReceiver();
    ~ImuReceiver();

    bool Init(movesense::Simou3Camera* cam, ImuQueue* queue, int pollMs, uint64_t minimumTimestampUs = 0);
};

} // namespace movesense_x95_ros
