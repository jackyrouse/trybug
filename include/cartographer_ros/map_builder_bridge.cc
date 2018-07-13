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

#include "cartographer_ros/map_builder_bridge.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/transform/transform.h"



namespace cartographer_ros
{
namespace
{

using ::cartographer::transform::Rigid3d;

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kLandmarkMarkerScale = 0.3;
constexpr double kConstraintMarkerScale = 0.025;

} // namespace

MapBuilderBridge::MapBuilderBridge() {}


void
MapBuilderBridge::HandlerSubmapQuery()
{

    ToGeometryMsgPose(cartographer::transform::Rigid3d());

    return;
}


MySubmapdata::MySubmapList
MapBuilderBridge::GetSubmapList()
{
    MySubmapdata::MySubmapList my_submap_list;

    ToGeometryMsgPose(cartographer::transform::Rigid3d());

    return my_submap_list;
}

} // namespace cartographer_ros
