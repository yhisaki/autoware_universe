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

/// One predicted ego state: the pose in map coordinates, velocity [m/s], and steering [rad].
struct EgoState
{
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  double velocity{0.0};
  double steering{0.0};
};

/**
 * @brief Restores normalized (x, y, cos(yaw), sin(yaw), velocity, steering) predictions to meters,
 * unit yaw vectors, and m/s.
 */
std::vector<float> denormalize_prediction(const std::vector<float> & prediction);

/**
 * @brief Parses denormalized ego-frame state predictions into ego states in map coordinates.
 *
 * @param prediction Denormalized states with shape (B, T, 6).
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @return A 2D vector structure: [batch][timestep] -> EgoState.
 */
std::vector<std::vector<EgoState>> parse_predictions(
  const std::vector<float> & prediction, const Eigen::Matrix4d & transform_ego_to_map);

/**
 * @brief Creates a Trajectory message from parsed ego states for a specific batch.
 *
 * The velocity and front wheel angle are the predicted ones, and the acceleration is the forward
 * difference of the velocity profile (0 on the last point). The heading rate is left at zero.
 * When the trajectory optimization is enabled it recomputes all of these consistently with the
 * vehicle model.
 *
 * @param ego_states The parsed ego states [batch][timestep].
 * @param stamp The ROS time stamp for the message.
 * @param batch_index The batch index to extract.
 * @return A Trajectory message for the ego agent in the specified batch.
 */
Trajectory create_ego_trajectory(
  const std::vector<std::vector<EgoState>> & ego_states, const rclcpp::Time & stamp,
  int64_t batch_index);

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
