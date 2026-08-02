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

#include "autoware/diffusion_planner/diffusion_planner_core.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/inference/guidance/centerline_guidance.hpp"
#include "autoware/diffusion_planner/inference/guidance/start_guidance.hpp"
#include "autoware/diffusion_planner/inference/guidance/stop_guidance.hpp"
#include "autoware/diffusion_planner/inference/multi_step_inference.hpp"
#include "autoware/diffusion_planner/inference/single_step_inference.hpp"
#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/diffusion_planner/preprocessing/items/sampled_trajectories.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#ifdef AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
#include "autoware/diffusion_planner/inference/onnxruntime_inference.hpp"
#endif

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
#ifdef AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
namespace
{
bool is_onnxruntime_backend(const std::string & backend)
{
  return backend == "ort_cpu" || backend == "ort_cuda" || backend == "ort_tensorrt";
}

std::string onnxruntime_execution_provider_from_backend(const std::string & backend)
{
  if (backend == "ort_cpu") {
    return "cpu";
  }
  if (backend == "ort_cuda") {
    return "cuda";
  }
  if (backend == "ort_tensorrt") {
    return "tensorrt";
  }
  throw std::invalid_argument(
    "Unsupported model.backend '" + backend +
    "'. Expected 'tensorrt', 'ort_cpu', 'ort_cuda', or 'ort_tensorrt'.");
}
}  // namespace
#endif

DiffusionPlannerCore::DiffusionPlannerCore(
  const DiffusionPlannerParams & params, const VehicleInfo & vehicle_info)
: params_(params), vehicle_spec_(vehicle_info)
{
  sync_turn_indicator_managers();
}

void DiffusionPlannerCore::sync_turn_indicator_managers()
{
  const auto hold_duration = rclcpp::Duration::from_seconds(params_.turn_indicator_hold_duration);
  const float keep_offset = params_.turn_indicator_keep_offset;
  const size_t desired = static_cast<size_t>(std::max<int>(params_.batch_size, 1));

  if (turn_indicator_managers_.size() > desired) {
    turn_indicator_managers_.erase(
      turn_indicator_managers_.begin() + static_cast<std::ptrdiff_t>(desired),
      turn_indicator_managers_.end());
  }
  while (turn_indicator_managers_.size() < desired) {
    turn_indicator_managers_.emplace_back(hold_duration, keep_offset);
  }
  for (auto & manager : turn_indicator_managers_) {
    manager.set_hold_duration(hold_duration);
    manager.set_keep_offset(keep_offset);
  }
}

void DiffusionPlannerCore::load_model()
{
  diffusion_planner_inference_.reset();
  utils::check_weight_version(params_.args_path);
  observation_normalization_ = utils::load_observation_normalization(params_.args_path);
  state_normalization_ = utils::load_state_normalization(params_.args_path);

  // Initialize guidance modules
  StartGuidanceConfig start_guidance_config;
  start_guidance_config.reference_distance_m =
    static_cast<float>(params_.start_guidance_reference_distance_m);
  start_guidance_config.max_scale = static_cast<float>(params_.start_guidance_max_scale);
  start_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
  start_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
  start_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
  start_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
  start_guidance_ = std::make_shared<StartGuidance>(start_guidance_config);
  start_guidance_->set_enabled(start_guidance_enabled_);

  StopGuidanceConfig stop_guidance_config;
  stop_guidance_config.stop_acceleration_mps2 =
    static_cast<float>(params_.stop_guidance_stop_acceleration_mps2);
  stop_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
  stop_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
  stop_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
  stop_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
  stop_guidance_ = std::make_shared<StopGuidance>(stop_guidance_config);
  stop_guidance_->set_enabled(stop_guidance_enabled_);

  CenterlineGuidanceConfig centerline_guidance_config;
  centerline_guidance_config.start_time_s =
    static_cast<float>(params_.centerline_guidance_start_time_s);
  centerline_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
  centerline_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
  centerline_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
  centerline_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
  centerline_guidance_ = std::make_shared<CenterlineGuidance>(centerline_guidance_config);
  centerline_guidance_->set_enabled(centerline_guidance_enabled_);

  std::unordered_map<std::string, std::shared_ptr<Guidance>> guidances{
    {"start", start_guidance_}, {"stop", stop_guidance_}, {"centerline", centerline_guidance_}};
  if (params_.backend == "tensorrt" && params_.model_type == "single_step") {
    diffusion_planner_inference_ = std::make_unique<SingleStepInference>(
      params_.single_step_model_path, params_.plugins_path, params_.batch_size,
      params_.trt_precision, params_.use_cuda_graph);
  } else if (params_.backend == "tensorrt" && params_.model_type == "multi_step") {
    diffusion_planner_inference_ = std::make_unique<MultiStepInference>(
      params_.encoder_model_path, params_.decoder_model_path, params_.turn_indicator_model_path,
      params_.plugins_path, params_.batch_size, params_.trt_precision, params_.use_cuda_graph,
      params_.dpm_solver_steps, std::move(guidances));
#ifdef AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
  } else if (is_onnxruntime_backend(params_.backend) && params_.model_type == "single_step") {
    diffusion_planner_inference_ = std::make_unique<OnnxruntimeSingleStepInference>(
      params_.single_step_model_path, onnxruntime_execution_provider_from_backend(params_.backend),
      params_.plugins_path, params_.batch_size);
  } else if (is_onnxruntime_backend(params_.backend) && params_.model_type == "multi_step") {
    diffusion_planner_inference_ = std::make_unique<OnnxruntimeMultiStepInference>(
      params_.encoder_model_path, params_.decoder_model_path, params_.turn_indicator_model_path,
      onnxruntime_execution_provider_from_backend(params_.backend), params_.plugins_path,
      params_.batch_size, params_.dpm_solver_steps, std::move(guidances));
#endif
  } else {
    if (params_.backend != "tensorrt") {
      throw std::invalid_argument(
        "Unsupported model.backend '" + params_.backend +
        "'. ONNX Runtime support is not available in this build.");
    }
    throw std::invalid_argument(
      "Unsupported model.type '" + params_.model_type +
      "'. Expected 'single_step' or 'multi_step'.");
  }
}

void DiffusionPlannerCore::update_params(const DiffusionPlannerParams & params)
{
  params_ = params;
  sync_turn_indicator_managers();
  if (start_guidance_) {
    StartGuidanceConfig start_guidance_config;
    start_guidance_config.reference_distance_m =
      static_cast<float>(params_.start_guidance_reference_distance_m);
    start_guidance_config.max_scale = static_cast<float>(params_.start_guidance_max_scale);
    start_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
    start_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
    start_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
    start_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
    start_guidance_->set_config(start_guidance_config);
    start_guidance_->set_enabled(start_guidance_enabled_);
  }
  if (stop_guidance_) {
    StopGuidanceConfig stop_guidance_config;
    stop_guidance_config.stop_acceleration_mps2 =
      static_cast<float>(params_.stop_guidance_stop_acceleration_mps2);
    stop_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
    stop_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
    stop_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
    stop_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
    stop_guidance_->set_config(stop_guidance_config);
    stop_guidance_->set_enabled(stop_guidance_enabled_);
  }
  if (centerline_guidance_) {
    CenterlineGuidanceConfig centerline_guidance_config;
    centerline_guidance_config.start_time_s =
      static_cast<float>(params_.centerline_guidance_start_time_s);
    centerline_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
    centerline_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
    centerline_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
    centerline_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
    centerline_guidance_->set_config(centerline_guidance_config);
    centerline_guidance_->set_enabled(centerline_guidance_enabled_);
  }
}

void DiffusionPlannerCore::set_start_guidance_enabled(const bool enabled)
{
  start_guidance_enabled_ = enabled;
  if (start_guidance_) {
    start_guidance_->set_enabled(enabled);
  }
}

void DiffusionPlannerCore::set_stop_guidance_enabled(const bool enabled)
{
  stop_guidance_enabled_ = enabled;
  if (stop_guidance_) {
    stop_guidance_->set_enabled(enabled);
  }
}

void DiffusionPlannerCore::set_centerline_guidance_enabled(const bool enabled)
{
  centerline_guidance_enabled_ = enabled;
  if (centerline_guidance_) {
    centerline_guidance_->set_enabled(enabled);
  }
}

void DiffusionPlannerCore::set_map(
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr)
{
  lane_segment_context_ = std::make_unique<preprocess::LaneSegmentContext>(
    lanelet_map_ptr, params_.line_string_max_step_m);
}

void DiffusionPlannerCore::update_buffers(
  const Odometry & ego_kinematic_state, const TrackedObjects & objects,
  const std::vector<std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
    traffic_signals,
  const TurnIndicatorsReport & turn_indicators, const LaneletRoute::ConstSharedPtr & route_ptr)
{
  route_ptr_ = route_ptr;
  ego_history_.push_back(ego_kinematic_state);
  turn_indicators_history_.push_back(turn_indicators);
  objects_history_.push_back(objects);
  for (const auto & msg : traffic_signals) {
    if (msg) {
      traffic_signals_history_.push_back(*msg);
    }
  }
}

preprocess::InputDataResult DiffusionPlannerCore::create_input_data(
  const std::shared_ptr<const Odometry> & ego_kinematic_state,
  const std::shared_ptr<const TrackedObjects> & objects,
  const std::vector<std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
    traffic_signals,
  const std::shared_ptr<const TurnIndicatorsReport> & turn_indicators,
  const LaneletRoute::ConstSharedPtr & route_ptr)
{
  const LaneletRoute::ConstSharedPtr effective_route_ptr = route_ptr ? route_ptr : route_ptr_;

  std::vector<std::string> missing_inputs;
  if (!ego_kinematic_state) {
    missing_inputs.emplace_back("ego kinematic state");
  }
  if (!objects) {
    missing_inputs.emplace_back("tracked objects");
  }
  if (!effective_route_ptr) {
    missing_inputs.emplace_back("route");
  }
  if (!turn_indicators) {
    missing_inputs.emplace_back("turn indicators");
  }
  if (!missing_inputs.empty()) {
    std::string error{"Missing required input: "};
    for (size_t index = 0; index < missing_inputs.size(); ++index) {
      if (index > 0) {
        error += ", ";
      }
      error += missing_inputs[index];
    }
    return tl::unexpected(std::move(error));
  }

  update_buffers(
    *ego_kinematic_state, *objects, traffic_signals, *turn_indicators, effective_route_ptr);

  if (!lane_segment_context_) {
    return tl::unexpected(std::string{"Map context is unavailable"});
  }

  if (stop_guidance_) {
    const auto & linear = ego_history_.back().twist.twist.linear;
    stop_guidance_->set_current_speed_mps(static_cast<float>(std::hypot(linear.x, linear.y)));
  }

  // Build the single-batch model inputs with the shared builder, used both
  // online (here) and offline (training data creation from rosbags).
  const preprocess::FrameInputs frame_inputs{
    frame_time(),
    ego_history_.msgs(),
    turn_indicators_history_.msgs(),
    objects_history_.msgs(),
    traffic_signals_history_.msgs(),
    *route_ptr_};
  const preprocess::InputBuilderParams builder_params{
    params_.traffic_light_group_msg_timeout_seconds};
  auto single_input_result = preprocess::create_input_data_map(
    frame_inputs, *lane_segment_context_, vehicle_spec_, builder_params, &selected_agents_);
  if (!single_input_result) {
    return tl::unexpected(single_input_result.error());
  }
  InputDataMap single_input_data_map = std::move(single_input_result.value());

  // Replicate for batch
  InputDataMap input_data_map;
  for (auto & [key, value] : single_input_data_map) {
    input_data_map[key] = utils::replicate_for_batch(value, params_.batch_size);
  }

  if (centerline_guidance_) {
    const auto & route_lanes = input_data_map.at("route_lanes");
    centerline_guidance_->set_route_lanes(
      std::vector<float>(route_lanes.cbegin(), route_lanes.cend()));
  }

  // Random sample trajectories (inference only)
  std::vector<size_t> sampled_shape;
  std::transform(
    SAMPLED_TRAJECTORIES_SHAPE.begin(), SAMPLED_TRAJECTORIES_SHAPE.end(),
    std::back_inserter(sampled_shape), [](const int64_t dim) { return static_cast<size_t>(dim); });
  sampled_shape.front() = static_cast<size_t>(params_.batch_size);
  xt::xarray<float> sampled_tensor = xt::xarray<float>::from_shape(sampled_shape);
  const size_t single_sample_size = sampled_tensor.size() / sampled_shape.front();
  for (int64_t b = 0; b < params_.batch_size; b++) {
    const xt::xarray<float> sampled_trajectories =
      preprocess::create_sampled_trajectories(params_.temperature_list[b]);
    std::copy(
      sampled_trajectories.begin(), sampled_trajectories.end(),
      sampled_tensor.begin() + static_cast<std::ptrdiff_t>(b * single_sample_size));
  }
  input_data_map["sampled_trajectories"] = std::move(sampled_tensor);

  return input_data_map;
}

rclcpp::Time DiffusionPlannerCore::frame_time() const
{
  return rclcpp::Time(ego_history_.back().header.stamp);
}

InferenceResult DiffusionPlannerCore::run_inference(const InputDataMap & input_data_map)
{
  if (!diffusion_planner_inference_) {
    return tl::unexpected(std::string{"Model not loaded"});
  }
  return diffusion_planner_inference_->infer(input_data_map);
}

PlannerOutput DiffusionPlannerCore::create_planner_output(
  const InferenceOutput & inference_output, const rclcpp::Time & timestamp,
  const UUID & generator_uuid)
{
  // Derive the frame state from the raw message buffers
  const Odometry & kinematic_state = ego_history_.back();
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(kinematic_state.pose.pose);

  const auto & [raw_predictions, turn_indicator_logit] = inference_output.outputs;
  const std::vector<float> denormalized_predictions =
    inference_output.is_denormalized
      ? raw_predictions
      : postprocess::denormalize_prediction(raw_predictions, state_normalization_);
  std::vector<float> denormalized_denoising_predictions;
  if (!inference_output.denoising_predictions.empty()) {
    denormalized_denoising_predictions =
      inference_output.is_denormalized
        ? inference_output.denoising_predictions
        : postprocess::denormalize_prediction(
            inference_output.denoising_predictions, state_normalization_, true);
  }

  const auto agent_poses =
    postprocess::parse_predictions(denormalized_predictions, ego_to_map_transform);

  const bool enable_force_stop =
    kinematic_state.twist.twist.linear.x > std::numeric_limits<double>::epsilon();

  PlannerOutput output;
  output.denoising_steps = postprocess::create_denoising_steps_message(
    denormalized_denoising_predictions, inference_output.denoising_timesteps);

  const int64_t prev_report = turn_indicators_history_.empty()
                                ? TurnIndicatorsReport::DISABLE
                                : turn_indicators_history_.back().report;

  // Trajectory and CandidateTrajectories
  for (int i = 0; i < params_.batch_size; i++) {
    auto trajectory = postprocess::create_ego_trajectory(
      agent_poses, timestamp, kinematic_state.pose.pose.position, i, enable_force_stop,
      params_.stopping_threshold);

    if (i == 0) {
      // Use the first trajectory as the main output trajectory
      output.trajectory = trajectory;
    }

    // TurnIndicatorsCommand
    const std::vector<float> single_turn_indicator_logit(
      turn_indicator_logit.begin() + TURN_INDICATOR_OUTPUT_DIM * i,
      turn_indicator_logit.begin() + TURN_INDICATOR_OUTPUT_DIM * (i + 1));
    const TurnIndicatorsCommand turn_indicators_command =
      turn_indicator_managers_.at(i).evaluate(single_turn_indicator_logit, timestamp, prev_report);

    if (i == 0) {
      // Publish the first trajectory's command on the standalone turn indicator topic.
      output.turn_indicators_command = turn_indicators_command;
    }

    autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_trajectory;
    candidate_trajectory.header = trajectory.header;
    candidate_trajectory.generator_id = generator_uuid;
    candidate_trajectory.points = trajectory.points;
    candidate_trajectory.turn_indicators_command = turn_indicators_command;

    std_msgs::msg::String generator_name_msg;
    generator_name_msg.data = std::string("DiffusionPlanner_batch_") + std::to_string(i);

    autoware_internal_planning_msgs::msg::GeneratorInfo generator_info;
    generator_info.generator_id = generator_uuid;
    generator_info.generator_name = generator_name_msg;

    output.candidate_trajectories.candidate_trajectories.push_back(candidate_trajectory);
    output.candidate_trajectories.generator_info.push_back(generator_info);
  }

  // PredictedObjects
  // Use the first prediction as the main predicted objects
  constexpr int64_t batch_idx = 0;
  output.predicted_objects =
    postprocess::create_predicted_objects(agent_poses, selected_agents_, timestamp, batch_idx);

  output.guidance_triggered = inference_output.guidance_triggered;

  return output;
}

autoware_perception_msgs::msg::TrafficLightGroup
DiffusionPlannerCore::get_first_traffic_light_on_route() const
{
  if (!lane_segment_context_ || !route_ptr_ || ego_history_.empty()) {
    return autoware_perception_msgs::msg::TrafficLightGroup{};
  }

  const geometry_msgs::msg::Pose & pose_center = ego_history_.back().pose.pose;

  const double center_x = pose_center.position.x;
  const double center_y = pose_center.position.y;
  const double center_z = pose_center.position.z;

  const auto traffic_light_id_map = preprocess::create_traffic_signal_map(
    traffic_signals_history_.msgs(), frame_time(), params_.traffic_light_group_msg_timeout_seconds);

  return lane_segment_context_->get_first_traffic_light_on_route(
    *route_ptr_, center_x, center_y, center_z, traffic_light_id_map);
}

int64_t DiffusionPlannerCore::count_valid_elements(
  const InputDataMap & input_data_map, const std::string & data_key) const
{
  const int64_t batch_idx = 0;

  if (data_key == "lanes") {
    return postprocess::count_valid_elements(
      input_data_map.at("lanes"), LANES_SHAPE[1], LANES_SHAPE[2], LANES_SHAPE[3], batch_idx);
  } else if (data_key == "route_lanes") {
    return postprocess::count_valid_elements(
      input_data_map.at("route_lanes"), ROUTE_LANES_SHAPE[1], ROUTE_LANES_SHAPE[2],
      ROUTE_LANES_SHAPE[3], batch_idx);
  } else if (data_key == "intersection_area") {
    return postprocess::count_valid_elements(
      input_data_map.at("intersection_area"), INTERSECTION_AREA_SHAPE[1],
      INTERSECTION_AREA_SHAPE[2], INTERSECTION_AREA_SHAPE[3], batch_idx);
  } else if (data_key == "stop_lines") {
    return postprocess::count_valid_elements(
      input_data_map.at("stop_lines"), STOP_LINES_SHAPE[1], STOP_LINES_SHAPE[2],
      STOP_LINES_SHAPE[3], batch_idx);
  } else if (data_key == "road_borders") {
    return postprocess::count_valid_elements(
      input_data_map.at("road_borders"), ROAD_BORDERS_SHAPE[1], ROAD_BORDERS_SHAPE[2],
      ROAD_BORDERS_SHAPE[3], batch_idx);
  } else if (data_key == "neighbor_agents_past") {
    return postprocess::count_valid_elements(
      input_data_map.at("neighbor_agents_past"), NEIGHBOR_SHAPE[1], NEIGHBOR_SHAPE[2],
      NEIGHBOR_SHAPE[3], batch_idx);
  }

  throw std::invalid_argument("Unknown data_key '" + data_key + "' in count_valid_elements()");
}

}  // namespace autoware::diffusion_planner
