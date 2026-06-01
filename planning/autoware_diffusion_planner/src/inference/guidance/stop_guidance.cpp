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

#include "autoware/diffusion_planner/inference/guidance/stop_guidance.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"
#include "utils.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::diffusion_planner
{

StopGuidance::StopGuidance()
{
  config_ = StopGuidanceConfig{};
}

StopGuidance::StopGuidance(const StopGuidanceConfig & config)
{
  config_ = config;
}

void StopGuidance::set_config(const StopGuidanceConfig & config)
{
  config_ = config;
}

void StopGuidance::set_current_speed_mps(const float current_speed_mps)
{
  current_speed_mps_ = current_speed_mps;
}

GuidanceResult StopGuidance::compute_delta(
  [[maybe_unused]] const GuidanceContext & context, const std::vector<float> & model_output) const
{
  GuidanceResult result;
  if (!is_enabled()) {
    return result;
  }

  const float stop_acceleration = config_.stop_acceleration_mps2;
  const std::vector<std::vector<Eigen::Vector4f>> trajectories =
    extract_denormalized_trajectories_from_model_output(
      model_output, config_.state_mean, config_.state_std);
  if (trajectories.empty()) {
    return result;
  }

  const float reference_distance =
    current_speed_mps_ * current_speed_mps_ / (2.0f * stop_acceleration);

  std::vector<std::vector<Eigen::Vector4f>> guided_trajectories = trajectories;
  result.triggered.assign(trajectories.size(), false);
  bool has_delta = false;

  for (int64_t b = 0; b < static_cast<int64_t>(trajectories.size()); ++b) {
    const auto & trajectory = trajectories[b];

    const float terminal_dx = trajectory.back().x() - trajectory.front().x();
    const float terminal_dy = trajectory.back().y() - trajectory.front().y();
    const float terminal_distance = std::hypot(terminal_dx, terminal_dy);
    if (
      !std::isfinite(reference_distance) || !std::isfinite(terminal_distance) ||
      terminal_distance <= 0.0 || terminal_distance <= reference_distance) {
      continue;
    }

    auto & guided_trajectory = guided_trajectories[b];
    const float scale = reference_distance / terminal_distance;

    for (int64_t t = 1; t <= OUTPUT_T; ++t) {
      guided_trajectory[t].x() =
        trajectory.front().x() + (trajectory[t].x() - trajectory.front().x()) * scale;
      guided_trajectory[t].y() =
        trajectory.front().y() + (trajectory[t].y() - trajectory.front().y()) * scale;
    }

    result.triggered[b] = true;
    has_delta = true;
  }

  if (!has_delta) {
    return result;
  }

  result.delta = create_delta_from_denormalized_trajectories(
    trajectories, guided_trajectories, config_.state_std);
  return result;
}

}  // namespace autoware::diffusion_planner
