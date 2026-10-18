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

TEST(PostprocessingUtilsTest, CreateTrajectoryFromStatePrediction)
{
  // Normalized states: x = t / 50 (1 m per step), unit heading, velocity 10 / 15, steering 0.1.
  std::vector<float> prediction(OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2], 0.0f);
  for (int64_t t = 0; t < OUTPUT_T; ++t) {
    float * state = prediction.data() + t * STATE_DIM;
    state[0] = static_cast<float>(t + 1) / POSITION_SCALE;
    state[2] = 2.0F;  // Not unit length: denormalization projects it onto the unit circle.
    state[STATE_IDX_VELOCITY] = 10.0F / SPEED_SCALE;
    state[STATE_IDX_STEERING] = 0.1F;
  }

  const auto ego_states = postprocess::parse_predictions(
    postprocess::denormalize_prediction(prediction), Eigen::Matrix4d::Identity());
  ASSERT_EQ(ego_states.size(), 1U);
  ASSERT_EQ(ego_states[0].size(), static_cast<size_t>(OUTPUT_T));
  EXPECT_NEAR(ego_states[0].back().pose(0, 3), static_cast<double>(OUTPUT_T), 1e-3);
  EXPECT_NEAR(ego_states[0].back().pose(0, 0), 1.0, 1e-6);

  const auto traj = postprocess::create_ego_trajectory(ego_states, rclcpp::Time(123, 0), 0);
  ASSERT_EQ(traj.points.size(), static_cast<size_t>(OUTPUT_T));
  EXPECT_NEAR(traj.points.front().longitudinal_velocity_mps, 10.0F, 1e-4F);
  EXPECT_NEAR(traj.points.front().front_wheel_angle_rad, 0.1F, 1e-6F);
  EXPECT_NEAR(traj.points.front().acceleration_mps2, 0.0F, 1e-3F);
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
// Ego states on the model output grid, laid out along +x with the given velocities.
std::vector<std::vector<postprocess::EgoState>> make_ego_states_along_x(
  const std::vector<double> & velocities)
{
  std::vector<postprocess::EgoState> ego_states;
  double x = 0.0;
  for (const double velocity : velocities) {
    x += 0.1 * velocity;
    postprocess::EgoState state;
    state.pose(0, 3) = x;
    state.velocity = velocity;
    ego_states.push_back(state);
  }
  return {ego_states};
}
}  // namespace

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryUsesPredictedVelocity)
{
  const auto ego_states = make_ego_states_along_x({10.0, 10.0, 10.0});

  const auto trajectory = postprocess::create_ego_trajectory(ego_states, rclcpp::Time(0), 0);

  ASSERT_EQ(trajectory.points.size(), 3U);
  for (const auto & point : trajectory.points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, 10.0F, 1e-3F);
    EXPECT_NEAR(point.acceleration_mps2, 0.0F, 1e-3F);
    EXPECT_FLOAT_EQ(point.heading_rate_rps, 0.0F);
  }
}

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryAcceleration)
{
  // 1, 2, 3 m/s on the 0.1 s grid: 10 m/s^2.
  const auto ego_states = make_ego_states_along_x({1.0, 2.0, 3.0});

  const auto trajectory = postprocess::create_ego_trajectory(ego_states, rclcpp::Time(0), 0);

  ASSERT_EQ(trajectory.points.size(), 3U);
  EXPECT_NEAR(trajectory.points[0].acceleration_mps2, 10.0F, 1e-2F);
  EXPECT_NEAR(trajectory.points[1].acceleration_mps2, 10.0F, 1e-2F);
  // The last point has no successor.
  EXPECT_FLOAT_EQ(trajectory.points[2].acceleration_mps2, 0.0F);
}

}  // namespace autoware::ml_planner::test
