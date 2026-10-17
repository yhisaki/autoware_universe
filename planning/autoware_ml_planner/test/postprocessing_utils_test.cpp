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

#include "autoware/ml_planner/postprocessing/postprocessing_utils.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace autoware::ml_planner::test
{
using autoware_planning_msgs::msg::Trajectory;

TEST(PostprocessingUtilsTest, CreateTrajectoryFromControls)
{
  // Zero normalized controls keep the current velocity and steering.
  const std::vector<float> controls(OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2], 0.0f);
  const auto poses =
    postprocess::rollout_controls(postprocess::denormalize_prediction(controls), 5.0, 0.0, 2.75);
  ASSERT_EQ(poses.size(), static_cast<size_t>(OUTPUT_T * POSE_DIM));
  // Straight at 5 m/s on the 0.1 s grid: the last point is 40 m ahead.
  EXPECT_NEAR(poses[(OUTPUT_T - 1) * POSE_DIM], 0.5 * OUTPUT_T, 1e-3);
  EXPECT_NEAR(poses[(OUTPUT_T - 1) * POSE_DIM + 1], 0.0, 1e-6);

  const auto ego_poses_batch = postprocess::parse_predictions(poses, Eigen::Matrix4d::Identity());
  geometry_msgs::msg::Point base_position;
  const auto traj =
    postprocess::create_ego_trajectory(ego_poses_batch, rclcpp::Time(123, 0), base_position, 0);
  ASSERT_EQ(traj.points.size(), static_cast<size_t>(OUTPUT_T));
  EXPECT_NEAR(traj.points.back().longitudinal_velocity_mps, 5.0F, 1e-3F);
}

TEST(PostprocessingUtilsTest, RolloutControlsTurnsWithSteeringRate)
{
  // A positive steering rate turns left: denormalized 0.3 rad/s.
  std::vector<float> controls(OUTPUT_T * CONTROL_DIM, 0.0f);
  for (int64_t t = 0; t < OUTPUT_T; ++t) controls[t * CONTROL_DIM + 1] = 1.0f;
  const auto poses =
    postprocess::rollout_controls(postprocess::denormalize_prediction(controls), 5.0, 0.0, 2.75);
  EXPECT_GT(poses[(OUTPUT_T - 1) * POSE_DIM + 1], 1.0F);
}

TEST(PostprocessingUtilsTest, FixStopPointsAfterConfiguredDecelerationDuration)
{
  Trajectory trajectory;
  for (size_t i = 0; i < 7; ++i) {
    auto & point = trajectory.points.emplace_back();
    const double time = 0.5 * static_cast<double>(i);
    point.time_from_start.sec = static_cast<int32_t>(time);
    point.time_from_start.nanosec = static_cast<uint32_t>((time - std::floor(time)) * 1.0e9);
    point.pose.position.x = static_cast<double>(i);
    point.longitudinal_velocity_mps = i >= 5 ? 0.2F : 2.0F;
    point.acceleration_mps2 = i >= 3 ? -1.0F : 0.0F;
  }

  postprocess::StopPointFixingParams params;
  params.velocity_threshold_mps = 0.3;
  params.min_deceleration_duration_sec = 1.0;
  const auto stop_index = postprocess::fix_stop_points(trajectory, params);

  ASSERT_EQ(stop_index, 5U);
  EXPECT_DOUBLE_EQ(trajectory.points[4].pose.position.x, 4.0);
  for (size_t i = 5; i < trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.points[i].pose.position.x, 5.0);
    EXPECT_FLOAT_EQ(trajectory.points[i].longitudinal_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(trajectory.points[i].acceleration_mps2, 0.0F);
  }
}

TEST(PostprocessingUtilsTest, FixStopPointsResetsDecelerationDuration)
{
  Trajectory trajectory;
  for (size_t i = 0; i < 5; ++i) {
    auto & point = trajectory.points.emplace_back();
    point.time_from_start.sec = static_cast<int32_t>(i);
    point.pose.position.x = static_cast<double>(i);
    point.longitudinal_velocity_mps = 0.2F;
    point.acceleration_mps2 = -1.0F;
  }
  trajectory.points[2].acceleration_mps2 = 0.0F;

  postprocess::StopPointFixingParams params;
  params.min_deceleration_duration_sec = 2.0;
  EXPECT_FALSE(postprocess::fix_stop_points(trajectory, params).has_value());
  EXPECT_DOUBLE_EQ(trajectory.points.back().pose.position.x, 4.0);
}

namespace
{
// Ego poses on the model output grid, laid out along +x at the given positions.
std::vector<std::vector<Eigen::Matrix4d>> make_ego_poses_along_x(
  const std::vector<double> & positions_x)
{
  std::vector<Eigen::Matrix4d> ego_poses;
  for (const double x : positions_x) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = x;
    ego_poses.push_back(pose);
  }
  return {ego_poses};
}
}  // namespace

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryConstantSpeed)
{
  // Points 1 m apart on the 0.1 s grid, the first one 1 m ahead of the ego position: 10 m/s.
  const auto ego_poses_batch = make_ego_poses_along_x({1.0, 2.0, 3.0, 4.0, 5.0});
  geometry_msgs::msg::Point base_position;

  const auto trajectory =
    postprocess::create_ego_trajectory(ego_poses_batch, rclcpp::Time(0), base_position, 0);

  ASSERT_EQ(trajectory.points.size(), 5U);
  for (const auto & point : trajectory.points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, 10.0F, 1e-3F);
    EXPECT_NEAR(point.acceleration_mps2, 0.0F, 1e-3F);
    // The model predicts poses only.
    EXPECT_FLOAT_EQ(point.heading_rate_rps, 0.0F);
    EXPECT_FLOAT_EQ(point.front_wheel_angle_rad, 0.0F);
  }
}

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryAcceleration)
{
  // Steps of 0.1, 0.2, 0.3 m on the 0.1 s grid: 1, 2, 3 m/s, i.e. 10 m/s^2.
  const auto ego_poses_batch = make_ego_poses_along_x({0.1, 0.3, 0.6});
  geometry_msgs::msg::Point base_position;

  const auto trajectory =
    postprocess::create_ego_trajectory(ego_poses_batch, rclcpp::Time(0), base_position, 0);

  ASSERT_EQ(trajectory.points.size(), 3U);
  EXPECT_NEAR(trajectory.points[0].longitudinal_velocity_mps, 1.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[1].longitudinal_velocity_mps, 2.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[2].longitudinal_velocity_mps, 3.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[0].acceleration_mps2, 10.0F, 1e-2F);
  EXPECT_NEAR(trajectory.points[1].acceleration_mps2, 10.0F, 1e-2F);
  // The last point has no successor.
  EXPECT_FLOAT_EQ(trajectory.points[2].acceleration_mps2, 0.0F);
}

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryUsesEgoPositionForTheFirstPoint)
{
  // Same poses, ego already at x = 1: the first point no longer covers any distance.
  const auto ego_poses_batch = make_ego_poses_along_x({1.0, 2.0});
  geometry_msgs::msg::Point base_position;
  base_position.x = 1.0;

  const auto trajectory =
    postprocess::create_ego_trajectory(ego_poses_batch, rclcpp::Time(0), base_position, 0);

  ASSERT_EQ(trajectory.points.size(), 2U);
  EXPECT_NEAR(trajectory.points[0].longitudinal_velocity_mps, 0.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[1].longitudinal_velocity_mps, 10.0F, 1e-3F);
}

}  // namespace autoware::ml_planner::test
