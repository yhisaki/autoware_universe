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

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ml_planner::postprocess
{
using autoware_planning_msgs::msg::TrajectoryPoint;

// internal functions
namespace
{
/**
 * @brief Converts predicted ego states to a Trajectory message.
 *
 * The velocity and front wheel angle are the predicted ones, and the acceleration is the forward
 * difference of the velocity profile. The heading rate stays at zero. The trajectory
 * optimization, when enabled, recomputes all of them consistently with the vehicle model.
 *
 * @param states Predicted ego states in map coordinates.
 * @param stamp The ROS time stamp for the message.
 * @return A Trajectory message in map coordinates.
 */
Trajectory get_trajectory_from_states(
  const std::vector<EgoState> & states, const rclcpp::Time & stamp);
};  // namespace

std::vector<float> denormalize_prediction(const std::vector<float> & prediction)
{
  const size_t trajectory_size = static_cast<size_t>(OUTPUT_T) * STATE_DIM;
  if (prediction.empty() || prediction.size() % trajectory_size != 0) {
    throw std::runtime_error("Unsupported prediction shape for trajectory denormalization.");
  }

  std::vector<float> denormalized = prediction;
  for (size_t index = 0; index < denormalized.size(); index += STATE_DIM) {
    denormalized[index] *= POSITION_SCALE;
    denormalized[index + 1] *= POSITION_SCALE;
    const float norm = std::hypot(denormalized[index + 2], denormalized[index + 3]);
    const float safe_norm = std::max(norm, 1.0e-6F);
    denormalized[index + 2] /= safe_norm;
    denormalized[index + 3] /= safe_norm;
    denormalized[index + STATE_IDX_VELOCITY] *= SPEED_SCALE;
  }
  return denormalized;
}

std::vector<std::vector<EgoState>> parse_predictions(
  const std::vector<float> & prediction, const Eigen::Matrix4d & transform_ego_to_map)
{
  const int64_t batch_size = prediction.size() / (OUTPUT_T * STATE_DIM);

  // Ensure prediction has enough data
  const size_t required_size = batch_size * OUTPUT_T * STATE_DIM;
  if (prediction.size() < required_size) {
    throw std::runtime_error(
      "Prediction vector size (" + std::to_string(prediction.size()) +
      ") is smaller than required (" + std::to_string(required_size) + ")");
  }

  // Structure: batch -> timestep -> state
  std::vector<std::vector<EgoState>> parsed_predictions(
    batch_size, std::vector<EgoState>(OUTPUT_T));

  for (int64_t batch_idx = 0; batch_idx < batch_size; ++batch_idx) {
    for (int64_t time_idx = 0; time_idx < OUTPUT_T; ++time_idx) {
      const int64_t pred_base_idx = (batch_idx * OUTPUT_T + time_idx) * STATE_DIM;

      const double x = static_cast<double>(prediction[pred_base_idx + 0]);
      const double y = static_cast<double>(prediction[pred_base_idx + 1]);
      const double cos_yaw = static_cast<double>(prediction[pred_base_idx + 2]);
      const double sin_yaw = static_cast<double>(prediction[pred_base_idx + 3]);

      // Create 4x4 transformation matrix from x, y, cos(yaw), sin(yaw)
      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      pose(0, 0) = cos_yaw;
      pose(0, 1) = -sin_yaw;
      pose(1, 0) = sin_yaw;
      pose(1, 1) = cos_yaw;
      pose(0, 3) = x;
      pose(1, 3) = y;

      auto & state = parsed_predictions[batch_idx][time_idx];
      state.pose = transform_ego_to_map * pose;
      state.velocity = static_cast<double>(prediction[pred_base_idx + STATE_IDX_VELOCITY]);
      state.steering = static_cast<double>(prediction[pred_base_idx + STATE_IDX_STEERING]);
    }
  }

  return parsed_predictions;
}

Trajectory create_ego_trajectory(
  const std::vector<std::vector<EgoState>> & ego_states, const rclcpp::Time & stamp,
  const int64_t batch_index)
{
  // Validate batch index
  if (batch_index < 0 || batch_index >= static_cast<int64_t>(ego_states.size())) {
    throw std::out_of_range(
      "Invalid batch_index: " + std::to_string(batch_index) +
      ", batch_size=" + std::to_string(ego_states.size()));
  }

  return get_trajectory_from_states(ego_states[batch_index], stamp);
}

int64_t count_valid_elements(
  const xt::xarray<float> & data, int64_t len, int64_t dim2, int64_t dim3, int64_t batch_idx)
{
  const int64_t single_batch_size = len * dim2 * dim3;
  const int64_t batch_offset = batch_idx * single_batch_size;

  if (batch_offset + single_batch_size > static_cast<int64_t>(data.size()) || batch_idx < 0) {
    return 0;  // Invalid batch index or data size
  }

  int64_t valid_count = 0;
  const float epsilon = std::numeric_limits<float>::epsilon();

  // Iterate through each element in the len dimension for the specified batch
  for (int64_t i = 0; i < len; ++i) {
    bool is_valid_element = false;

    // Check all values in the (dim2, dim3) block for this element
    const int64_t element_offset = batch_offset + i * dim2 * dim3;
    for (int64_t j = 0; j < dim2 * dim3; ++j) {
      const int64_t idx = element_offset + j;
      if (std::abs(data.data()[idx]) > epsilon) {
        is_valid_element = true;
        break;  // Found non-zero value, element is valid
      }
    }

    if (is_valid_element) {
      valid_count++;
    }
  }

  return valid_count;
}

std::optional<size_t> fix_stop_points(Trajectory & trajectory, const StopPointFixingParams & params)
{
  auto & points = trajectory.points;
  auto stop_it = points.end();
  auto deceleration_start_it = points.end();
  for (auto it = points.begin(); it != points.end(); ++it) {
    if (it->acceleration_mps2 >= -0.01F) {
      deceleration_start_it = points.end();
      continue;
    }

    if (deceleration_start_it == points.end()) {
      deceleration_start_it = it;
    }
    const auto & start_time = deceleration_start_it->time_from_start;
    const auto & current_time = it->time_from_start;
    const double deceleration_duration = static_cast<double>(current_time.sec - start_time.sec) +
                                         1.0e-9 * static_cast<double>(
                                                    static_cast<int64_t>(current_time.nanosec) -
                                                    static_cast<int64_t>(start_time.nanosec));
    if (
      deceleration_duration >= params.min_deceleration_duration_sec &&
      it->longitudinal_velocity_mps <= params.velocity_threshold_mps) {
      stop_it = it;
      break;
    }
  }
  if (stop_it == points.end()) {
    return std::nullopt;
  }

  const auto stop_pose = stop_it->pose;
  const float stop_steering = stop_it->front_wheel_angle_rad;
  for (auto it = stop_it; it != points.end(); ++it) {
    it->pose = stop_pose;
    it->longitudinal_velocity_mps = 0.0F;
    it->lateral_velocity_mps = 0.0F;
    it->acceleration_mps2 = 0.0F;
    it->heading_rate_rps = 0.0F;
    it->front_wheel_angle_rad = stop_steering;
  }
  return static_cast<size_t>(std::distance(points.begin(), stop_it));
}

namespace
{
Trajectory get_trajectory_from_states(
  const std::vector<EgoState> & states, const rclcpp::Time & stamp)
{
  Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = "map";
  constexpr double dt = 0.1;

  for (size_t i = 0; i < states.size(); ++i) {
    const double curr_time = dt * static_cast<double>(i + 1);
    TrajectoryPoint p;
    p.time_from_start.sec = static_cast<int>(curr_time);
    p.time_from_start.nanosec = static_cast<uint32_t>((curr_time - p.time_from_start.sec) * 1e9);

    // Extract position from transformation matrix
    const auto & pose = states[i].pose;
    p.pose.position.x = pose(0, 3);
    p.pose.position.y = pose(1, 3);
    p.pose.position.z = pose(2, 3);

    // Extract 3x3 rotation matrix and convert to quaternion
    const Eigen::Matrix3d rotation_matrix = pose.block<3, 3>(0, 0);
    const Eigen::Quaterniond quaternion(rotation_matrix);
    p.pose.orientation.x = quaternion.x();
    p.pose.orientation.y = quaternion.y();
    p.pose.orientation.z = quaternion.z();
    p.pose.orientation.w = quaternion.w();

    p.longitudinal_velocity_mps = static_cast<float>(states[i].velocity);
    p.front_wheel_angle_rad = static_cast<float>(states[i].steering);

    trajectory.points.push_back(p);
  }

  // Acceleration as the forward difference of the velocity profile; the last point has no
  // successor and keeps zero.
  for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
    const double v0 = trajectory.points[i].longitudinal_velocity_mps;
    const double v1 = trajectory.points[i + 1].longitudinal_velocity_mps;
    trajectory.points[i].acceleration_mps2 = static_cast<float>((v1 - v0) / dt);
  }

  return trajectory;
}

}  // namespace

}  // namespace autoware::ml_planner::postprocess
