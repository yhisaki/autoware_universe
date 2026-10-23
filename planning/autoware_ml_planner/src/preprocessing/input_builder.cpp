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

#include "autoware/ml_planner/preprocessing/input_builder.hpp"

#include "autoware/ml_planner/constants.hpp"
#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/preprocessing/items/agent.hpp"
#include "autoware/ml_planner/preprocessing/items/ego_history.hpp"
#include "autoware/ml_planner/preprocessing/items/ego_shape.hpp"
#include "autoware/ml_planner/preprocessing/items/goal_pose.hpp"
#include "autoware/ml_planner/preprocessing/items/traffic_signals.hpp"
#include "autoware/ml_planner/preprocessing/items/turn_indicators.hpp"
#include "autoware/ml_planner/utils/utils.hpp"

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ml_planner::preprocess
{
std::unique_ptr<LaneSegmentContext> build_map_context(
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr, double line_string_max_step_m)
{
  return std::make_unique<LaneSegmentContext>(lanelet_map_ptr, line_string_max_step_m);
}

InputBuilderResult create_input_data_map(
  const FrameInputs & frame_inputs, const LaneSegmentContext & map_context,
  const VehicleSpec & vehicle_spec, const InputBuilderParams & params)
{
  if (frame_inputs.ego_history.empty()) {
    return tl::unexpected(std::string{"Ego history is empty"});
  }
  if (frame_inputs.route.segments.empty()) {
    return tl::unexpected(std::string{"Route has no segments"});
  }

  const nav_msgs::msg::Odometry & kinematic_state = frame_inputs.ego_history.back();
  const geometry_msgs::msg::Pose & pose_center = kinematic_state.pose.pose;
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(pose_center);
  const Eigen::Matrix4d map_to_ego_transform = utils::inverse(ego_to_map_transform);
  const auto center_x = static_cast<float>(pose_center.position.x);
  const auto center_y = static_cast<float>(pose_center.position.y);
  const auto center_z = static_cast<float>(pose_center.position.z);
  const double center_yaw = utils::yaw_from_quaternion(pose_center.orientation);

  // Ego
  EgoData ego{
    extract_ego_history(
      frame_inputs.ego_history, frame_inputs.steering_history, EGO_HISTORY_SHAPE[1],
      map_to_ego_transform, frame_inputs.frame_time),
    extract_turn_indicators(
      frame_inputs.turn_indicators_history, frame_inputs.frame_time, INPUT_T_WITH_CURRENT,
      constants::PREDICTION_TIME_STEP_S),
    vehicle_spec, extract_goal_pose(frame_inputs.route.goal_pose, map_to_ego_transform)};

  // Neighbor agents on ego reference frame
  AgentData agents;
  agents.selected = select_current_agents(
    frame_inputs.objects_history, frame_inputs.frame_time, map_to_ego_transform, MAX_NUM_NEIGHBORS);
  agents.past_poses = extract_neighbor_agent_sequence(
    frame_inputs.objects_history, agents.selected, frame_inputs.frame_time, map_to_ego_transform,
    MAX_NUM_NEIGHBORS, INPUT_T + 1, constants::PREDICTION_TIME_STEP_S,
    AgentSequenceDirection::Past);

  // Lane data with the traffic light state history of each selected lane segment
  const auto build_lane_data =
    [&](const std::vector<int64_t> & segment_indices, const int64_t max_segments) {
      LaneData lane_data =
        map_context.create_lane_data(map_to_ego_transform, segment_indices, max_segments);
      lane_data.traffic_light_past = extract_traffic_light_past(
        frame_inputs.traffic_signals_history, map_context.get_traffic_light_ids(segment_indices),
        max_segments, frame_inputs.frame_time, INPUT_T_WITH_CURRENT,
        constants::PREDICTION_TIME_STEP_S, params.traffic_light_group_msg_timeout_seconds);
      return lane_data;
    };

  MapData map;
  map.lanes = build_lane_data(
    map_context.select_lane_segment_indices(
      map_to_ego_transform, center_x, center_y, NUM_SEGMENTS_IN_LANE),
    NUM_SEGMENTS_IN_LANE);
  map.route_lanes = build_lane_data(
    map_context.select_route_segment_indices(
      frame_inputs.route, center_x, center_y, center_z, center_yaw, NUM_SEGMENTS_IN_ROUTE),
    NUM_SEGMENTS_IN_ROUTE);

  // Intersection areas, stop lines, and road borders
  map.intersection_areas =
    map_context.create_intersection_area_tensor(map_to_ego_transform, center_x, center_y);
  map.stop_lines = map_context.create_stop_line_tensor(map_to_ego_transform, center_x, center_y);
  map.road_borders =
    map_context.create_road_border_tensor(map_to_ego_transform, center_x, center_y);

  return InputBuilderOutput{std::move(ego), std::move(agents), std::move(map)};
}

TensorMap InputBuilderOutput::get_tensor() const
{
  TensorMap input_data_map;

  // Ego
  input_data_map["ego_agent_past"] = create_ego_history_tensor(ego.history);
  input_data_map["ego_shape"] = create_ego_shape(
    ego.vehicle_spec.base_link_to_front, ego.vehicle_spec.vehicle_length,
    ego.vehicle_spec.vehicle_width, ego.vehicle_spec.wheel_base);
  input_data_map["turn_indicators"] = create_turn_indicators_tensor(ego.turn_indicators);
  input_data_map["goal_pose"] = create_goal_pose_tensor(ego.goal_pose);

  // Neighbor agents
  input_data_map["neighbor_agents_past"] =
    create_neighbor_agent_sequence_tensor(agents.past_poses, MAX_NUM_NEIGHBORS, INPUT_T + 1);
  input_data_map["agent_shape"] = create_agent_shape(agents.selected, MAX_NUM_NEIGHBORS);
  input_data_map["agent_label"] = create_agent_label(agents.selected, MAX_NUM_NEIGHBORS);

  // Lanes and route lanes
  // e.g. lane_key "lane" -> "lanes", "lane_types", "lanes_speed_limit";
  // traffic_light_key "lane" -> "lane_traffic_light_past", "lane_traffic_light_future"
  const auto add_lane_tensors = [&](
                                  const LaneData & lane_data, const std::string & lane_key,
                                  const std::string & traffic_light_key,
                                  const int64_t max_segments) {
    input_data_map[lane_key + "s"] = create_lane_input_tensor(lane_data.points);
    input_data_map[lane_key + "_types"] =
      create_lane_type_tensor(lane_data.line_types, max_segments);
    input_data_map[lane_key + "s_speed_limit"] =
      create_speed_limit_tensor(lane_data.speed_limits_mps, max_segments);
    xt::xarray<float> traffic_light_past = create_traffic_light_past_tensor(
      lane_data.traffic_light_past, max_segments, INPUT_T_WITH_CURRENT);
    input_data_map[traffic_light_key + "_traffic_light_future"] =
      infer_traffic_light_future(traffic_light_past);
    input_data_map[traffic_light_key + "_traffic_light_past"] = std::move(traffic_light_past);
  };
  add_lane_tensors(map.lanes, "lane", "lane", NUM_SEGMENTS_IN_LANE);
  add_lane_tensors(map.route_lanes, "route_lane", "route", NUM_SEGMENTS_IN_ROUTE);

  // Intersection areas, stop lines, and road borders
  input_data_map["intersection_area"] = create_polyline_input_tensor(map.intersection_areas);
  input_data_map["stop_lines"] = create_polyline_input_tensor(map.stop_lines);
  input_data_map["road_borders"] = create_polyline_input_tensor(map.road_borders);

  return input_data_map;
}

}  // namespace autoware::ml_planner::preprocess
