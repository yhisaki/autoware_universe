// Copyright 2026 TIER IV, Inc.
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

#include "utils.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

size_t trajectory_index(
  const int64_t batch, const int64_t agent, const int64_t t, const int64_t dim)
{
  return ((static_cast<size_t>(batch) * MAX_NUM_AGENTS + agent) * (OUTPUT_T + 1) + t) * POSE_DIM +
         dim;
}

std::vector<std::vector<Eigen::Vector4f>> extract_denormalized_trajectories_from_model_output(
  const std::vector<float> & model_output, const Eigen::Vector4f & state_mean,
  const Eigen::Vector4f & state_std)
{
  const size_t trajectory_size = static_cast<size_t>(MAX_NUM_AGENTS) * (OUTPUT_T + 1) * POSE_DIM;
  if (trajectory_size == 0 || model_output.empty() || model_output.size() % trajectory_size != 0) {
    return {};
  }

  const int64_t batch_size = static_cast<int64_t>(model_output.size() / trajectory_size);
  std::vector<std::vector<Eigen::Vector4f>> trajectories;
  trajectories.reserve(batch_size);

  for (int64_t b = 0; b < batch_size; ++b) {
    std::vector<Eigen::Vector4f> trajectory;
    trajectory.reserve(OUTPUT_T + 1);

    Eigen::Vector4f current_state(0.0f, 0.0f, 1.0f, 0.0f);
    for (int64_t dim = 2; dim < POSE_DIM; ++dim) {
      current_state[dim] =
        model_output[trajectory_index(b, k_ego_agent_index, 0, dim)] * state_std[dim] +
        state_mean[dim];
    }
    trajectory.push_back(current_state);

    for (int64_t t = 1; t <= OUTPUT_T; ++t) {
      Eigen::Vector4f point;
      for (int64_t dim = 0; dim < POSE_DIM; ++dim) {
        point[dim] = model_output[trajectory_index(b, k_ego_agent_index, t, dim)] * state_std[dim] +
                     state_mean[dim];
      }
      trajectory.push_back(point);
    }

    trajectories.push_back(std::move(trajectory));
  }

  return trajectories;
}

std::vector<float> create_delta_from_denormalized_trajectories(
  const std::vector<std::vector<Eigen::Vector4f>> & trajectories,
  const std::vector<std::vector<Eigen::Vector4f>> & guided_trajectories,
  const Eigen::Vector4f & state_std)
{
  const size_t trajectory_size = static_cast<size_t>(MAX_NUM_AGENTS) * (OUTPUT_T + 1) * POSE_DIM;
  if (trajectory_size == 0 || trajectories.empty()) {
    return {};
  }

  const int64_t batch_size = static_cast<int64_t>(trajectories.size());
  if (guided_trajectories.size() != trajectories.size()) {
    return {};
  }

  for (int64_t b = 0; b < batch_size; ++b) {
    const auto & trajectory = trajectories[b];
    const auto & guided_trajectory = guided_trajectories[b];
    if (
      trajectory.size() != static_cast<size_t>(OUTPUT_T + 1) ||
      guided_trajectory.size() != trajectory.size()) {
      return {};
    }
  }

  std::vector<float> delta(static_cast<size_t>(batch_size) * trajectory_size, 0.0f);
  for (int64_t b = 0; b < batch_size; ++b) {
    const auto & trajectory = trajectories[b];
    const auto & guided_trajectory = guided_trajectories[b];
    for (int64_t t = 0; t <= OUTPUT_T; ++t) {
      for (int64_t dim = 0; dim < POSE_DIM; ++dim) {
        const size_t idx = trajectory_index(b, k_ego_agent_index, t, dim);
        delta[idx] = (guided_trajectory[t][dim] - trajectory[t][dim]) / state_std[dim];
      }
    }
  }

  return delta;
}

}  // namespace autoware::diffusion_planner
