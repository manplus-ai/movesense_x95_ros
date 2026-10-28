// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/stereo_publisher.h"

#include "movesense_x95/seg_result.h"

#include <cmath>
#include <cstring>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>

namespace movesense_x95_ros {

StereoPublisher::StereoPublisher() {}

StereoPublisher::~StereoPublisher() {}

bool StereoPublisher::Init(ros::NodeHandle& nh, StereoFrameQueue* frameQueue, const std::string& cam0Topic, const std::string& cam1Topic,
    const std::string& colorTopic, const std::string& depthTopic, const std::string& detTopic, const std::string& cam0FrameId,
    const std::string& cam1FrameId, const std::string& colorFrameId, const std::string& depthFrameId, const std::string& detFrameId, int queueSize,
    bool enableStereo, bool enableColor, bool enableDetection, bool enableDepth)
{
    if (frameQueue == nullptr || queueSize <= 0 || m_running.load() || m_thread.joinable()) {
        ROS_ERROR("[StereoPublisher] Init rejected invalid argument or active publisher");
        return false;
    }

    m_frameQueue = frameQueue;
    m_cam0FrameId = cam0FrameId;
    m_cam1FrameId = cam1FrameId;
    m_colorFrameId = colorFrameId;
    m_depthFrameId = depthFrameId;
    m_detFrameId = detFrameId;
    m_stereoEnabled = enableStereo;
    m_colorEnabled = enableColor;
    m_depthEnabled = enableDepth;
    m_detEnabled = enableDetection;

    if (m_stereoEnabled) {
        m_cam0Pub = nh.advertise<sensor_msgs::Image>(cam0Topic, queueSize);
        m_cam1Pub = nh.advertise<sensor_msgs::Image>(cam1Topic, queueSize);
    }
    if (m_depthEnabled) {
        m_depthPub = nh.advertise<sensor_msgs::Image>(depthTopic, queueSize);
    } else {
        m_depthInfoPub.shutdown();
        m_hasDepthInfo = false;
    }
    if (m_colorEnabled) {
        m_colorPub = nh.advertise<sensor_msgs::Image>(colorTopic, queueSize);
    }
    if (m_detEnabled) {
        m_detPub = nh.advertise<vision_msgs::Detection2DArray>(detTopic, queueSize);
    }

    m_running.store(true);
    m_thread = std::thread(&StereoPublisher::Loop, this);

    return true;
}

void StereoPublisher::Shutdown()
{
    m_running.store(false);
    if (m_frameQueue != nullptr) {
        m_frameQueue->Stop();
    }

    if (m_thread.joinable()) {
        m_thread.join();
    }
    m_frameQueue = nullptr;
    m_cam0Pub.shutdown();
    m_cam1Pub.shutdown();
    m_colorPub.shutdown();
    m_colorInfoPub.shutdown();
    m_depthPub.shutdown();
    m_depthInfoPub.shutdown();
    m_detPub.shutdown();
    m_hasDepthInfo = false;
    m_hasColorInfo = false;
}

bool StereoPublisher::SetDepthUnitMm(double unitMm)
{
    if (!std::isfinite(unitMm) || unitMm <= 0.0 || m_running.load() || m_thread.joinable()) {
        ROS_ERROR("[StereoPublisher] Invalid depth unit or active publisher");
        return false;
    }

    m_depthUnitMm = unitMm;
    return true;
}

void StereoPublisher::SetImageGeometry(const CameraConfig& cfg, bool isPassive)
{
    if (m_running.load() || m_thread.joinable()) {
        ROS_ERROR("[StereoPublisher] Image geometry cannot change while publishing");
        return;
    }

    const RoiConfig& color = cfg.roi[5];
    MakeImageGeometry(cfg.colorWidth, cfg.colorHeight, color.enable, color.x1, color.y1, color.x2, color.y2, m_colorGeometry);
    m_detectionSourceRight = isPassive;
    if (isPassive) {
        const RoiConfig& right = cfg.roi[4];
        MakeImageGeometry(cfg.stereoWidth, cfg.stereoHeight, right.enable, right.x1, right.y1, right.x2, right.y2, m_detectionGeometry);
    } else {
        m_detectionGeometry = m_colorGeometry;
    }
}

void StereoPublisher::SetDepthCameraInfo(ros::NodeHandle& nh, const std::string& topic, const sensor_msgs::CameraInfo& info, int queueSize)
{
    m_depthInfo = info;
    m_depthInfoPub = nh.advertise<sensor_msgs::CameraInfo>(topic, queueSize);
    m_hasDepthInfo = true;
}

void StereoPublisher::SetColorCameraInfo(ros::NodeHandle& nh, const std::string& topic, const sensor_msgs::CameraInfo& info, int queueSize)
{
    m_colorInfo = info;
    m_colorInfoPub = nh.advertise<sensor_msgs::CameraInfo>(topic, queueSize);
    m_hasColorInfo = true;
}

static ros::Time StampFromPts(uint64_t ptsUs)
{
    if (ptsUs == 0) {
        return ros::Time();
    }

    ros::Time stamp;
    stamp.fromNSec(static_cast<uint64_t>(ptsUs) * 1000);

    return stamp;
}

void StereoPublisher::PublishOne(ros::Publisher& pub, const PlaneData& img, const std::string& frameId)
{
    if (!img.valid || img.ptsUs == 0) {
        return;
    }

    sensor_msgs::Image msg;
    msg.header.stamp = StampFromPts(img.ptsUs);
    msg.header.frame_id = frameId;
    msg.is_bigendian = 0;

    if (img.format == static_cast<uint8_t>(1)) {
        const size_t needNv21 = static_cast<size_t>(img.width) * img.height * 3 / 2;
        if (img.data.size() < needNv21) {
            return;
        }

        cv::Mat nv21(img.height * 3 / 2, img.width, CV_8UC1, const_cast<uint8_t*>(img.data.data()));
        cv::Mat bgr;
        cv::cvtColor(nv21, bgr, cv::COLOR_YUV2BGR_NV21);

        msg.height = bgr.rows;
        msg.width = bgr.cols;
        msg.encoding = "bgr8";
        msg.step = bgr.cols * 3;
        msg.data.assign(bgr.data, bgr.data + static_cast<size_t>(bgr.cols) * bgr.rows * 3);
    } else {
        const size_t needY = static_cast<size_t>(img.width) * img.height;
        if (img.data.size() < needY) {
            return;
        }

        msg.height = img.height;
        msg.width = img.width;
        msg.encoding = "mono8";
        msg.step = img.width;
        msg.data.assign(img.data.begin(), img.data.begin() + needY);
    }

    pub.publish(msg);
}

void StereoPublisher::PublishColor(const PlaneData& color)
{
    if (!m_colorEnabled || !color.valid || color.ptsUs == 0 || color.width == 0 || color.height == 0 || color.width % 2 != 0 ||
        color.height % 2 != 0) {
        return;
    }

    const size_t needNv21 = static_cast<size_t>(color.width) * color.height * 3 / 2;

    if (color.data.size() < needNv21) {
        return;
    }

    cv::Mat nv21(color.height * 3 / 2, color.width, CV_8UC1, const_cast<uint8_t*>(color.data.data()));
    cv::Mat bgr;
    cv::cvtColor(nv21, bgr, cv::COLOR_YUV2BGR_NV21);

    sensor_msgs::Image msg;
    msg.header.stamp = StampFromPts(color.ptsUs);
    msg.header.frame_id = m_colorFrameId;
    msg.height = bgr.rows;
    msg.width = bgr.cols;
    msg.is_bigendian = 0;
    msg.encoding = "bgr8";
    msg.step = bgr.cols * 3;
    msg.data.assign(bgr.data, bgr.data + static_cast<size_t>(bgr.cols) * bgr.rows * 3);

    m_colorPub.publish(msg);

    if (m_hasColorInfo) {
        sensor_msgs::CameraInfo info = m_colorInfo;
        if (info.width == m_colorGeometry.fullWidth && info.height == m_colorGeometry.fullHeight &&
            AdjustCameraModel(m_colorGeometry, msg.width, msg.height, info.K.data(), info.P.data())) {
            info.header.stamp = msg.header.stamp;
            info.width = msg.width;
            info.height = msg.height;
            info.roi = sensor_msgs::RegionOfInterest();
            m_colorInfoPub.publish(info);
        } else {
            ROS_WARN_THROTTLE(5.0, "[StereoPublisher] Color camera_info does not match image geometry");
        }
    }
}

void StereoPublisher::PublishDepth(const PlaneData& depth)
{
    if (!m_depthEnabled || !depth.valid || depth.ptsUs == 0) {
        return;
    }

    size_t need = 0;
    if (!ImageByteSize(depth.width, depth.height, 2, need) || depth.data.size() < need) {
        return;
    }

    sensor_msgs::Image msg;
    msg.header.stamp = StampFromPts(depth.ptsUs);
    msg.header.frame_id = m_depthFrameId;
    msg.height = depth.height;
    msg.width = depth.width;
    msg.is_bigendian = 0;
    msg.encoding = "16UC1";
    msg.step = depth.width * 2;
    if (!ConvertDepthToMillimeters(depth.data, need / 2, m_depthUnitMm, msg.data)) {
        return;
    }

    m_depthPub.publish(msg);

    if (m_hasDepthInfo) {
        if (m_depthInfo.width == msg.width && m_depthInfo.height == msg.height) {
            m_depthInfo.header.stamp = msg.header.stamp;
            m_depthInfoPub.publish(m_depthInfo);
        } else {
            ROS_WARN_THROTTLE(5.0, "[StereoPublisher] Depth camera_info does not match image dimensions");
        }
    }
}

void StereoPublisher::PublishDetections(const PlaneData& seg, const PlaneData& source)
{
    if (!m_detEnabled || !seg.valid || seg.ptsUs == 0) {
        return;
    }

    if (!SegValid(seg.data.data(), seg.data.size())) {
        return;
    }

    SegHeader header;
    std::memcpy(&header, seg.data.data(), sizeof(header));
    ImageGeometry geometry;
    if (!ResolveImageGeometry(m_detectionGeometry, source.valid ? source.width : 0, source.valid ? source.height : 0, geometry)) {
        ROS_WARN_THROTTLE(5.0, "[StereoPublisher] Detection source image geometry is invalid");
        return;
    }

    vision_msgs::Detection2DArray arr;
    arr.header.stamp = StampFromPts(seg.ptsUs);
    arr.header.frame_id = m_detFrameId;
    arr.detections.reserve(header.detectionCount);

    for (uint16_t i = 0; i < header.detectionCount; ++i) {
        SegObject object;
        std::memcpy(&object, seg.data.data() + header.detectionsOffset + static_cast<size_t>(i) * sizeof(object), sizeof(object));
        DetectionRectangle rectangle;
        if (!std::isfinite(object.score) || object.score < 0.0f || object.score > 1.0f ||
            !MapDetectionRectangle(geometry, header.coordW, header.coordH, object.x1, object.y1, object.x2, object.y2, rectangle)) {
            continue;
        }

        vision_msgs::Detection2D d;
        d.header = arr.header;
        d.bbox.center.x = (rectangle.x1 + rectangle.x2) * 0.5;
        d.bbox.center.y = (rectangle.y1 + rectangle.y2) * 0.5;
        d.bbox.size_x = rectangle.x2 - rectangle.x1;
        d.bbox.size_y = rectangle.y2 - rectangle.y1;

        vision_msgs::ObjectHypothesisWithPose hyp;
        hyp.id = object.classId;
        hyp.score = object.score;
        d.results.push_back(hyp);

        arr.detections.push_back(std::move(d));
    }

    m_detPub.publish(arr);
}

void StereoPublisher::Loop()
{
    while (m_running.load()) {
        StereoFrame payload;

        if (!m_frameQueue->Pop(payload)) {
            break;
        }

        if (m_stereoEnabled) {
            PublishOne(m_cam0Pub, payload.right, m_cam0FrameId);
            PublishOne(m_cam1Pub, payload.left, m_cam1FrameId);
        }
        PublishColor(payload.color);
        PublishDepth(payload.depth);
        PublishDetections(payload.seg, m_detectionSourceRight ? payload.right : payload.color);
    }
}

} // namespace movesense_x95_ros
