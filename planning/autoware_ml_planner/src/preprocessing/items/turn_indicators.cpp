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

#include "autoware/ml_planner/preprocessing/items/turn_indicators.hpp"

#include <xtensor/xbuilder.hpp>

#include <deque>
#include <vector>

namespace autoware::ml_planner::preprocess
{
std::vector<uint8_t> extract_turn_indicators(
  const MessageView<autoware_vehicle_msgs::msg::TurnIndicatorsReport> & turn_indicators_msgs,
  const rclcpp::Time & current_time, const int64_t num_timesteps, const double time_step_s)
{
  std::vector<uint8_t> turn_indicators(static_cast<size_t>(num_timesteps), 0);
  if (turn_indicators_msgs.empty()) return turn_indicators;

  const double current_sec = current_time.seconds();
  constexpr double stamp_tolerance_s = 1e-6;
  size_t msg_idx = 0;
  for (int64_t t = 0; t < num_timesteps; ++t) {
    const double grid_sec = current_sec - static_cast<double>(num_timesteps - 1 - t) * time_step_s;
    while (msg_idx + 1 < turn_indicators_msgs.size() &&
           rclcpp::Time(turn_indicators_msgs[msg_idx + 1].stamp).seconds() <=
             grid_sec + stamp_tolerance_s) {
      ++msg_idx;
    }
    turn_indicators[t] = turn_indicators_msgs[msg_idx].report;
  }
  return turn_indicators;
}

xt::xarray<float> create_turn_indicators_tensor(const std::vector<uint8_t> & turn_indicators)
{
  xt::xarray<float> tensor = xt::zeros<float>({turn_indicators.size()});
  for (size_t t = 0; t < turn_indicators.size(); ++t) {
    tensor(t) = static_cast<float>(turn_indicators[t]);
  }
  return tensor;
}
}  // namespace autoware::ml_planner::preprocess
