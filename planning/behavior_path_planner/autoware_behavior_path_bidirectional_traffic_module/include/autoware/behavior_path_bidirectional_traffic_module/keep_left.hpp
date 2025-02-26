// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__KEEP_LEFT_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__KEEP_LEFT_HPP_

#include "autoware/behavior_path_bidirectional_traffic_module/connected_bidirectional_lanelets.hpp"
#include "autoware/trajectory/forward.hpp"

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <vector>

namespace autoware::behavior_path_planner
{

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
shift_trajectory_for_keep_left(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<ConnectedBidirectionalLanelets> & all_connected_bidirectional_lanelets,
  const double & keep_left_ratio, const double & vehicle_width);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__KEEP_LEFT_HPP_
