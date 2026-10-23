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

#include "autoware/ml_planner/utils/marker_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace autoware::ml_planner::test
{

using autoware::ml_planner::utils::create_lane_marker;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::MarkerArray;

TEST(MarkerUtilsTest, CreateLaneMarkerBasic)
{
  // Lane tensor: 1 segment, 2 points, (x, y, z) of center, left bound and right bound
  xt::xarray<float> lane_tensor = {
    {{1.0f, 2.0f, 0.0f, 1.5f, 2.5f, 0.0f, 0.5f, 1.5f, 0.0f},
     {2.0f, 3.0f, 0.0f, 2.5f, 3.5f, 0.0f, 1.5f, 2.5f, 0.0f}}};
  rclcpp::Time stamp(123456, 789, RCL_ROS_TIME);
  rclcpp::Duration lifetime(1, 0);
  Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  auto marker_array = create_lane_marker(identity, lane_tensor, stamp, lifetime);

  // Should create at least 1 marker for the centerline, and possibly for bounds/spheres
  EXPECT_GE(marker_array.markers.size(), 1u);

  for (const auto & marker : marker_array.markers) {
    EXPECT_EQ(marker.header.stamp, stamp);
    EXPECT_EQ(marker.header.frame_id, "base_link");
    EXPECT_GT(marker.points.size(), 0u);
    EXPECT_GT(marker.color.a, 0.0f);
  }
}

}  // namespace autoware::ml_planner::test
