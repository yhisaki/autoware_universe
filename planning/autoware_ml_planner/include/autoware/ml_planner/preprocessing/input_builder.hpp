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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__INPUT_BUILDER_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__INPUT_BUILDER_HPP_

#include "autoware/ml_planner/preprocessing/items/agent.hpp"
#include "autoware/ml_planner/preprocessing/items/ego_history.hpp"
#include "autoware/ml_planner/preprocessing/items/map.hpp"
#include "autoware/ml_planner/preprocessing/message_view.hpp"
#include "autoware/ml_planner/preprocessing/preprocessing_utils.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>
#include <tl/expected.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace autoware::ml_planner
{

/**
 * @brief Static ego vehicle specification derived from VehicleInfo.
 */
struct VehicleSpec
{
  double base_link_to_front;
  double vehicle_length;
  double vehicle_width;
  double wheel_base;

  VehicleSpec(
    const double base_link_to_front, const double vehicle_length, const double vehicle_width,
    const double wheel_base)
  : base_link_to_front(base_link_to_front),
    vehicle_length(vehicle_length),
    vehicle_width(vehicle_width),
    wheel_base(wheel_base)
  {
  }

  explicit VehicleSpec(const autoware::vehicle_info_utils::VehicleInfo & info)
  : base_link_to_front(info.wheel_base_m + info.front_overhang_m),
    vehicle_length(info.front_overhang_m + info.wheel_base_m + info.rear_overhang_m),
    vehicle_width(info.left_overhang_m + info.wheel_tread_m + info.right_overhang_m),
    wheel_base(info.wheel_base_m)
  {
  }
};

namespace preprocess
{

/**
 * @brief All per-frame inputs required to build the model input tensors.
 *
 * Holds raw message windows only: no derived or incrementally-updated state.
 * The same FrameInputs (plus map context and vehicle spec) always yields the
 * same TensorMap, whether filled online from subscriptions or offline from
 * a rosbag.
 */
struct FrameInputs
{
  rclcpp::Time frame_time;  ///< Reference time (newest sample of every history grid).
  MessageView<nav_msgs::msg::Odometry> ego_history;
  MessageView<autoware_vehicle_msgs::msg::SteeringReport> steering_history;
  MessageView<autoware_vehicle_msgs::msg::TurnIndicatorsReport> turn_indicators_history;
  MessageView<autoware_perception_msgs::msg::TrackedObjects> objects_history;
  MessageView<autoware_perception_msgs::msg::TrafficLightGroupArray> traffic_signals_history;
  const autoware_planning_msgs::msg::LaneletRoute & route;
};

struct InputBuilderParams
{
  double traffic_light_group_msg_timeout_seconds{0.2};
};

/**
 * @brief Ego vehicle data. Poses are in the ego frame at frame_inputs.frame_time.
 */
struct EgoData
{
  std::vector<EgoHistoryPoint> history;  ///< INPUT_T + 1 timesteps, oldest first
  std::vector<uint8_t> turn_indicators;  ///< Report value per timestep, INPUT_T + 1 timesteps
  VehicleSpec vehicle_spec;
  Eigen::Matrix4d goal_pose{Eigen::Matrix4d::Identity()};
};

/**
 * @brief Neighbor agents selected for the model input. Row i of every member is the same agent.
 */
struct AgentData
{
  std::vector<SelectedAgent> selected;
  std::vector<AgentPoseSequence> past_poses;  ///< INPUT_T + 1 timesteps per agent, oldest first
};

/**
 * @brief Map elements selected for the model input, as ego-frame (x, y, z) points.
 *
 * The z is dropped for the model, but kept here for visualization.
 */
struct MapData
{
  LaneData lanes;        ///< At most NUM_SEGMENTS_IN_LANE segments
  LaneData route_lanes;  ///< At most NUM_SEGMENTS_IN_ROUTE segments
  /// [NUM_INTERSECTION_AREAS, POINTS_PER_INTERSECTION_AREA, POLYLINE_3D_POINT_DIM]
  xt::xarray<float> intersection_areas;
  /// [NUM_STOP_LINES, POINTS_PER_STOP_LINE, POLYLINE_3D_POINT_DIM]
  xt::xarray<float> stop_lines;
  /// [NUM_ROAD_BORDERS, POINTS_PER_ROAD_BORDER, POLYLINE_3D_POINT_DIM]
  xt::xarray<float> road_borders;
};

/**
 * @brief Model input data of one frame, before conversion to tensors.
 */
struct InputBuilderOutput
{
  EgoData ego;
  AgentData agents;
  MapData map;

  /**
   * @brief Convert to a single-batch TensorMap with every model input except "initial_noise"
   * (inference-only random noise). No normalization and no batch replication is applied.
   */
  TensorMap get_tensor() const;
};

using InputBuilderResult = tl::expected<InputBuilderOutput, std::string>;

/**
 * @brief Build the map context from a lanelet map (stage 1, once per map).
 *
 * The returned context is immutable and deterministically derived from the
 * map; share it across all frames that use the same map.
 */
std::unique_ptr<LaneSegmentContext> build_map_context(
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr,
  double line_string_max_step_m);

/**
 * @brief Build the model input data for one frame (stage 2, pure function).
 *
 * Use InputBuilderOutput::get_tensor() to get the model input tensors.
 * All features are expressed in the ego frame at frame_inputs.frame_time.
 */
InputBuilderResult create_input_data_map(
  const FrameInputs & frame_inputs, const LaneSegmentContext & map_context,
  const VehicleSpec & vehicle_spec, const InputBuilderParams & params);

}  // namespace preprocess
}  // namespace autoware::ml_planner
#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__INPUT_BUILDER_HPP_
