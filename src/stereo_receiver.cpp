// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/stereo_receiver.h"

#include <chrono>
#include <movesense/Simou3Camera.h>
#include <ros/ros.h>

using namespace movesense;

namespace movesense_x95_ros {

StereoReceiver::StereoReceiver() {}

StereoReceiver::~StereoReceiver() {}

bool StereoReceiver::Init(Simou3Camera* cam, StereoFrameQueue* frameQueue, int timeoutMs)
{
    if (cam == nullptr || frameQueue == nullptr) {
        ROS_ERROR("[StereoReceiver] Init null argument");
        return false;
    }

    m_cam = cam;
    m_frameQueue = frameQueue;
    m_timeoutMs = timeoutMs;

    m_running.store(true);
    m_thread = std::thread(&StereoReceiver::Loop, this);

    return true;
}

void StereoReceiver::Shutdown()
{
    m_running.store(false);

    if (m_thread.joinable()) {
        m_thread.join();
    }
}

static void CopyPlane(const MovesenseFrame::Plane& src, PlaneData& dst)
{
    dst.valid = false;

    if (!src.has || !src.complete() || src.data.empty()) {
        return;
    }

    if (src.width == 0 || src.height == 0) {
        return;
    }

    dst.width = src.width;
    dst.height = src.height;
    dst.format = static_cast<uint8_t>(src.format);
    dst.ptsUs = src.pts;
    dst.data = src.data;
    dst.valid = true;
}

static void CopySeg(const MovesenseFrame::Plane& src, PlaneData& dst)
{
    dst.valid = false;

    if (!src.has || !src.complete() || src.data.empty()) {
        return;
    }

    dst.ptsUs = src.pts;
    dst.data = src.data;
    dst.valid = true;
}

void StereoReceiver::Loop()
{
    auto winStart = std::chrono::steady_clock::now();
    int recvCount = 0;

    while (m_running.load()) {
        MovesenseFrame frame;

        if (m_cam->getFrame(frame, m_timeoutMs)) {
            StereoFrame payload;
            payload.frameCnt = frame.frameCnt();

            CopyPlane(frame.leftRect(), payload.left);
            CopyPlane(frame.rightRect(), payload.right);
            CopyPlane(frame.rgbRect(), payload.color);
            CopyPlane(frame.depth(), payload.depth);
            CopySeg(frame.seg(), payload.seg);

            if (payload.left.valid || payload.right.valid || payload.color.valid || payload.depth.valid || payload.seg.valid) {
                m_frameQueue->Push(std::move(payload));
                ++recvCount;
            }
        }

        auto now = std::chrono::steady_clock::now();
        double sec = std::chrono::duration<double>(now - winStart).count();

        if (sec >= 5.0) {
            ROS_INFO("[rate] stereo %.1f fps (%d frames / %.1fs)", recvCount / sec, recvCount, sec);

            winStart = now;
            recvCount = 0;
        }
    }
}

} // namespace movesense_x95_ros
