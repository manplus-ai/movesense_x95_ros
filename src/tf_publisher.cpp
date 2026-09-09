// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros/tf_publisher.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

namespace movesense_x95_ros {

static std::string BodyFrameOf(const std::string& opticalFrameId)
{
    const std::string suffix = "_optical_frame";

    if (opticalFrameId.size() > suffix.size() && opticalFrameId.compare(opticalFrameId.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return opticalFrameId.substr(0, opticalFrameId.size() - suffix.size()) + "_frame";
    }

    return opticalFrameId + "_frame";
}

static geometry_msgs::TransformStamped MakeTransform(
    const ros::Time& stamp, const std::string& parent, const std::string& child, double tx, double ty, double tz, const tf2::Quaternion& q)
{
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = tx;
    tf.transform.translation.y = ty;
    tf.transform.translation.z = tz;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    return tf;
}

TfPublisher::TfPublisher() {}

TfPublisher::~TfPublisher() {}

bool TfPublisher::Publish(const CameraConfig& cfg, const StereoCalib& calib, bool colorFrameUsed, bool isPassive, int cameraType)
{
    m_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

    const ros::Time stamp = ros::Time::now();
    const tf2::Quaternion identity(0.0, 0.0, 0.0, 1.0);
    const tf2::Quaternion bodyToOptical(-0.5, 0.5, -0.5, 0.5);
    const std::string infra1Body = BodyFrameOf(cfg.cam1FrameId);

    std::vector<geometry_msgs::TransformStamped> transforms;

    transforms.push_back(MakeTransform(stamp, cfg.baseFrameId, infra1Body, 0.0, 0.0, 0.0, identity));
    transforms.push_back(MakeTransform(stamp, infra1Body, cfg.cam1FrameId, 0.0, 0.0, 0.0, bodyToOptical));

    const double baselineM = calib.BaselineMm() / 1000.0;
    if (calib.IsValid() && baselineM > 0.001) {
        transforms.push_back(MakeTransform(stamp, cfg.cam1FrameId, cfg.cam0FrameId, baselineM, 0.0, 0.0, identity));
    } else {
        ROS_WARN("[TfPublisher] invalid baseline (%.4f m); right camera TF not published", baselineM);
    }

    if (colorFrameUsed) {
        if (calib.HasColorExtrinsic()) {
            const std::array<double, 3>& treg = calib.ColorTregMm();
            transforms.push_back(
                MakeTransform(stamp, cfg.cam1FrameId, cfg.colorFrameId, -treg[0] / 1000.0, -treg[1] / 1000.0, -treg[2] / 1000.0, identity));
        } else {
            ROS_WARN("[TfPublisher] no valid color extrinsic in calibration; color TF not published");
        }
    } else if (!isPassive) {
        ROS_WARN("[TfPublisher] color frame not used (color stream off and depth not registered); color TF not published");
    }

    const bool imuOnColor = (cameraType == 0);
    const bool imuAvailable = imuOnColor ? calib.HasColorImuExtrinsic() : calib.HasImuExtrinsic();
    const std::string& imuParent = imuOnColor ? cfg.colorFrameId : cfg.cam1FrameId;

    if (imuAvailable && imuOnColor && !colorFrameUsed) {
        ROS_WARN("[TfPublisher] IMU is calibrated against the color camera but the color frame is not in the tree; IMU TF not published");
    } else if (imuAvailable) {
        const std::array<double, 9>& r = imuOnColor ? calib.ColorImuRotation() : calib.ImuRotation();
        const std::array<double, 3>& t = imuOnColor ? calib.ColorImuTranslationMm() : calib.ImuTranslationMm();

        tf2::Matrix3x3 rot(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
        tf2::Quaternion q;
        rot.getRotation(q);

        transforms.push_back(MakeTransform(stamp, imuParent, cfg.imuFrameId, t[0] / 1000.0, t[1] / 1000.0, t[2] / 1000.0, q));
        ROS_INFO("[TfPublisher] IMU frame attached to %s", imuParent.c_str());
    } else {
        ROS_WARN("[TfPublisher] no IMU extrinsic against %s in calibration; IMU TF not published", imuParent.c_str());
    }

    m_broadcaster->sendTransform(transforms);

    ROS_INFO("[TfPublisher] published %zu static transforms under %s", transforms.size(), cfg.baseFrameId.c_str());

    return true;
}

} // namespace movesense_x95_ros
