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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H

#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
/*
#include "cartographer_ros_msgs/LandmarkList.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"
*/

#include "IMU_PandCspace/imudata.h"
#include "LASER_PandCspace/laserdata.h"
#include "MySubmapdata/Submapdata.h"

namespace cartographer_ros
{

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const LASER_PandCspace::LASERMessage &msg);



Eigen::Vector3d ToEigen(const IMU_PandCspace::Vector3& vector3);

Eigen::Quaterniond ToEigen(const MySubmapdata::Quaternion& quaternion);

cartographer::transform::Rigid3d ToRigid3d(const MySubmapdata::Pose& pose);

cartographer::transform::Rigid3d ToRigid3d(const Eigen::Vector3d& vector, const Eigen::Quaterniond& quaternion);


Eigen::Vector3d
LatLongAltToEcef(double latitude, double longitude,
                 double altitude);


cartographer::transform::Rigid3d
ComputeLocalFrameFromLatLong(double latitude,
                             double longitude);


MySubmapdata::Point ToGeometryMsgPoint(const Eigen::Vector3d &vector3d);

MySubmapdata::Pose ToGeometryMsgPose(const ::cartographer::transform::Rigid3d &rigid3d);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H
