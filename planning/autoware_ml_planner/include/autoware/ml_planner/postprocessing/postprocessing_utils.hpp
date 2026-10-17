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

#ifndef AUTOWARE__ML_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_
#define AUTOWARE__ML_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_

#include "autoware/ml_planner/preprocessing/items/agent.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <xtensor/xarray.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cassert>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ml_planner::postprocess
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using unique_identifier_msgs::msg::UUID;

/**
 * @brief Parses ego poses into pose matrices in map coordinates.
 *
 * @param prediction Ego-frame poses (x, y, cos(yaw), sin(yaw) for each timestep), as returned by
 * rollout_controls.
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @return A 2D vector structure: [batch][timestep] -> Eigen::Matrix4d (4x4 pose matrix).
 */
std::vector<std::vector<Eigen::Matrix4d>> parse_predictions(
  const std::vector<float> & prediction, const Eigen::Matrix4d & transform_ego_to_map);

/**
 * @brief Restores normalized (acceleration, steering rate) controls to m/s^2 and rad/s.
 */
std::vector<float> denormalize_prediction(const std::vector<float> & prediction);

/**
 * @brief Integrates denormalized controls into ego-frame poses with the kinematic bicycle model
 * used for the training labels.
 *
 * @param controls Denormalized (acceleration, steering rate) controls with shape (B, T, 2).
 * @param initial_velocity Current ego velocity [m/s], as fed to the model.
 * @param initial_steering Current steering tire angle [rad], as fed to the model.
 * @param wheel_base Ego wheel base [m].
 * @return Ego-frame poses (x, y, cos(yaw), sin(yaw)) with shape (B, T, 4).
 */
std::vector<float> rollout_controls(
  const std::vector<float> & controls, double initial_velocity, double initial_steering,
  double wheel_base);

/**
 * @brief Creates a Trajectory message from parsed ego poses for a specific batch.
 *
 * The velocity of each point is its distance to the previous point divided by the time step,
 * with the current ego position as the predecessor of the first point, and the acceleration is
 * the forward difference of that speed profile (0 on the last point). The model predicts poses
 * only, so the heading rate and steering angle are left at zero. When the trajectory
 * optimization is enabled it recomputes all of these consistently with the vehicle model.
 *
 * @param ego_poses The parsed ego poses [batch][timestep] -> pose matrix.
 * @param stamp The ROS time stamp for the message.
 * @param base_position The current ego position in map coordinates.
 * @param batch_index The batch index to extract.
 * @return A Trajectory message for the ego agent in the specified batch.
 */
Trajectory create_ego_trajectory(
  const std::vector<std::vector<Eigen::Matrix4d>> & ego_poses, const rclcpp::Time & stamp,
  const geometry_msgs::msg::Point & base_position, int64_t batch_index);

/**
 * @brief Counts valid elements in a tensor with shape (B, len, dim2, dim3).
 * An element is considered valid if not all values in the (dim2, dim3) block are zero.
 *
 * @param data The input tensor data.
 * @param len The length dimension.
 * @param dim2 The second-to-last dimension.
 * @param dim3 The last dimension.
 * @param batch_idx The batch index to examine (0-based).
 * @return The number of valid elements in the specified batch.
 */
int64_t count_valid_elements(
  const xt::xarray<float> & data, int64_t len, int64_t dim2, int64_t dim3, int64_t batch_idx);

struct StopPointFixingParams
{
  bool enable{false};
  double velocity_threshold_mps{0.3};
  double min_deceleration_duration_sec{1.0};
};

/**
 * @brief Freeze the trajectory tail at the first stopping point.
 *
 * Finds the first point whose velocity is at or below the threshold after acceleration has
 * stayed negative for the configured duration. A non-decelerating point resets the duration.
 * The stop point and all subsequent points are fixed to its pose with zero velocity,
 * acceleration, and heading rate (steering angle is kept).
 *
 * @param trajectory Trajectory to modify in place.
 * @param params Stop detection parameters.
 * @return Index of the stop point, or std::nullopt when no stopping point was found.
 */
std::optional<size_t> fix_stop_points(
  Trajectory & trajectory, const StopPointFixingParams & params);

}  // namespace autoware::ml_planner::postprocess
#endif  // AUTOWARE__ML_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_
