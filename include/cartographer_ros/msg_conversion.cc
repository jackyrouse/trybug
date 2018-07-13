/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/msg_conversion.h"

#include <cmath>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/time_conversion.h"
/*
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
 */
#include "glog/logging.h"
/*
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"
 */

namespace cartographer_ros
{

constexpr float kPointCloudComponentFourMagic = 1.;

std::tuple<::cartographer::sensor::PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LASER_PandCspace::LASERMessage &msg)
{
    CHECK_GE(msg.range_min, 0.f);
    CHECK_GE(msg.range_max, msg.range_min);
    if (msg.angle_increment > 0.f)
    {
        CHECK_GT(msg.angle_max, msg.angle_min);
    }
    else
    {
        CHECK_GT(msg.angle_min, msg.angle_max);
    }
    ::cartographer::sensor::PointCloudWithIntensities point_cloud;
    float angle = msg.angle_min;
    for (size_t i = 0; i < msg.rangessize; ++i)
    {
        const auto &echoes = msg.ranges[i];
        const float first_echo = echoes;
        if (msg.range_min <= first_echo && first_echo <= msg.range_max)
        {
            const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
            Eigen::Vector4f point;
            point << rotation * (first_echo * Eigen::Vector3f::UnitX()), i * msg.time_increment;
            point_cloud.points.push_back(point);
            if (msg.intensitiessize > 0)
            {
                CHECK_EQ(msg.intensitiessize, msg.rangessize);
                const auto &echo_intensities = msg.intensities[i];
                point_cloud.intensities.push_back(echo_intensities);
            }
            else
            {
                point_cloud.intensities.push_back(0.f);
            }
        }
        angle += msg.angle_increment;
    }
    ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
    if (!point_cloud.points.empty())
    {
        const double duration = point_cloud.points.back()[3];
        timestamp += cartographer::common::FromSeconds(duration);
        for (Eigen::Vector4f &point : point_cloud.points)
        {
            point[3] -= duration;
        }
    }
    return std::make_tuple(point_cloud, timestamp);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const LASER_PandCspace::LASERMessage &msg)
{
    return LaserScanToPointCloudWithIntensities(msg);
}


Eigen::Vector3d ToEigen(const IMU_PandCspace::Vector3& vector3)
{
    return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const MySubmapdata::Quaternion& quaternion)
{
    return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                              quaternion.z);
}

cartographer::transform::Rigid3d ToRigid3d(const MySubmapdata::Pose& pose)
{
    return cartographer::transform::Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                   ToEigen(pose.orientation));
}

cartographer::transform::Rigid3d ToRigid3d(const Eigen::Vector3d& vector,const Eigen::Quaterniond& quaternion)
{
    return cartographer::transform::Rigid3d(vector, quaternion);
}

Eigen::Vector3d
LatLongAltToEcef(const double latitude, const double longitude,
                 const double altitude)
{
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
    constexpr double a = 6378137.; // semi-major axis, equator to center.
    constexpr double f = 1. / 298.257223563;
    constexpr double b = a * (1. - f); // semi-minor axis, pole to center.
    constexpr double a_squared = a * a;
    constexpr double b_squared = b * b;
    constexpr double e_squared = (a_squared - b_squared) / a_squared;
    const double sin_phi = std::sin(cartographer::common::DegToRad(latitude));
    const double cos_phi = std::cos(cartographer::common::DegToRad(latitude));
    const double sin_lambda = std::sin(cartographer::common::DegToRad(longitude));
    const double cos_lambda = std::cos(cartographer::common::DegToRad(longitude));
    const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
    const double x = (N + altitude) * cos_phi * cos_lambda;
    const double y = (N + altitude) * cos_phi * sin_lambda;
    const double z = (b_squared / a_squared * N + altitude) * sin_phi;

    return Eigen::Vector3d(x, y, z);
}

cartographer::transform::Rigid3d
ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude)
{
    const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(cartographer::common::DegToRad(latitude - 90.),
                          Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(cartographer::common::DegToRad(-longitude),
                              Eigen::Vector3d::UnitZ());
    return cartographer::transform::Rigid3d(rotation * -translation, rotation);
}


MySubmapdata::Point ToGeometryMsgPoint(const Eigen::Vector3d &vector3d)
{
    MySubmapdata::Point point;
    point.x = vector3d.x();
    point.y = vector3d.y();
    point.z = vector3d.z();
    return point;
}

MySubmapdata::Pose ToGeometryMsgPose(const cartographer::transform::Rigid3d &rigid3d)
{
    MySubmapdata::Pose pose;
    pose.position = ToGeometryMsgPoint(rigid3d.translation());
    pose.orientation.w = rigid3d.rotation().w();
    pose.orientation.x = rigid3d.rotation().x();
    pose.orientation.y = rigid3d.rotation().y();
    pose.orientation.z = rigid3d.rotation().z();
    return pose;
}
} // namespace cartographer_ros
