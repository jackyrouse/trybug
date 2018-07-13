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

#include "cartographer/common/make_unique.h"


#include "cartographer_ros/tf_bridge.h"


namespace cartographer_ros
{

TfBridge::TfBridge(const std::string &tracking_frame,
                   const double lookup_transform_timeout_sec)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec)
{}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time,
    const std::string &frame_id) const
{

    const Eigen::Vector3d imu_translation;
    const Eigen::Vector3d laser_translation;
    const Eigen::Quaterniond imu_quaternion;
    const Eigen::Quaterniond laser_quaternion;
    if(0 == frame_id.compare("imu"))
    {
        return ::cartographer::common::make_unique<::cartographer::transform::Rigid3d>(ToRigid3d(imu_translation, imu_quaternion));
    }
    if(0 == frame_id.compare("horizonal_2d_laser"))
    {
        return ::cartographer::common::make_unique<::cartographer::transform::Rigid3d>(ToRigid3d(laser_translation, laser_quaternion));
    }
    return nullptr;
}

} // namespace cartographer_ros
