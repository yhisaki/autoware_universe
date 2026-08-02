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

#ifndef AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__ITEMS__SAMPLED_TRAJECTORIES_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__ITEMS__SAMPLED_TRAJECTORIES_HPP_

#include <xtensor/xarray.hpp>

namespace autoware::diffusion_planner::preprocess
{
xt::xarray<float> create_sampled_trajectories(double temperature);
}  // namespace autoware::diffusion_planner::preprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__ITEMS__SAMPLED_TRAJECTORIES_HPP_
