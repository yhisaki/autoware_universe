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

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/inference/diffusion_planner.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/utils/arg_reader.hpp"
#include "cnpy.h"

#include <autoware/pyplot/pyplot.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using autoware::diffusion_planner::INPUT_T;
using autoware::diffusion_planner::LANES_SHAPE;
using autoware::diffusion_planner::LANES_SPEED_LIMIT_SHAPE;
using autoware::diffusion_planner::LINE_STRINGS_SHAPE;
using autoware::diffusion_planner::MAX_NUM_AGENTS;
using autoware::diffusion_planner::MAX_NUM_NEIGHBORS;
using autoware::diffusion_planner::OUTPUT_T;
using autoware::diffusion_planner::POLYGONS_SHAPE;
using autoware::diffusion_planner::POSE_DIM;
using autoware::diffusion_planner::ROUTE_LANES_SHAPE;
using autoware::diffusion_planner::ROUTE_LANES_SPEED_LIMIT_SHAPE;
using autoware::diffusion_planner::preprocess::InputDataMap;

template <class Container>
size_t num_elements_without_batch(const Container & shape)
{
  return std::accumulate(shape.begin() + 1, shape.end(), size_t{1}, std::multiplies<>());
}

struct Options
{
  std::string npz_path;
  std::string encoder_path;
  std::string decoder_path;
  std::string turn_indicator_path;
  std::string args_path;
  std::string plugins_path;
  std::string output_path;
  int batch_size{1};
  int steps{10};
  double temperature{0.5};
  bool overlay{false};
};

void print_usage(const char * program)
{
  std::cerr << "Usage: " << program << " --npz INPUT.npz --encoder ENCODER.onnx "
            << "--decoder DECODER.onnx --turn-indicator TURN_INDICATOR.onnx --args args.json "
            << "[--plugins plugins.so] [--batch-size 1] [--temperature 0.5] "
            << "[--steps 10] [--overlay] [--output figure.png]\n";
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    const auto require_value = [&](const char * name) {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + name);
      }
      return std::string(argv[++i]);
    };

    if (key == "--npz") {
      options.npz_path = require_value("--npz");
    } else if (key == "--encoder") {
      options.encoder_path = require_value("--encoder");
    } else if (key == "--decoder") {
      options.decoder_path = require_value("--decoder");
    } else if (key == "--turn-indicator") {
      options.turn_indicator_path = require_value("--turn-indicator");
    } else if (key == "--args") {
      options.args_path = require_value("--args");
    } else if (key == "--plugins") {
      options.plugins_path = require_value("--plugins");
    } else if (key == "--batch-size") {
      options.batch_size = std::stoi(require_value("--batch-size"));
    } else if (key == "--steps") {
      options.steps = std::stoi(require_value("--steps"));
    } else if (key == "--temperature") {
      options.temperature = std::stod(require_value("--temperature"));
    } else if (key == "--output") {
      options.output_path = require_value("--output");
    } else if (key == "--overlay") {
      options.overlay = true;
    } else if (key == "--help" || key == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown option: " + key);
    }
  }

  if (
    options.npz_path.empty() || options.encoder_path.empty() || options.decoder_path.empty() ||
    options.turn_indicator_path.empty() || options.args_path.empty()) {
    print_usage(argv[0]);
    throw std::runtime_error("Missing required option.");
  }
  if (options.batch_size <= 0) {
    throw std::runtime_error("--batch-size must be positive.");
  }
  if (options.steps < 2) {
    throw std::runtime_error("--steps must be greater than or equal to 2.");
  }
  return options;
}

std::vector<float> load_array_as_float(const cnpy::npz_t & npz, const std::string & key)
{
  const auto it = npz.find(key);
  if (it == npz.end()) {
    throw std::runtime_error("Missing npz key: " + key);
  }

  const auto & array = it->second;
  if (array.word_size == sizeof(float)) {
    return array.as_vec<float>();
  }
  if (array.word_size == sizeof(double)) {
    const auto values = array.as_vec<double>();
    return std::vector<float>(values.begin(), values.end());
  }
  if (array.word_size == sizeof(uint8_t)) {
    const auto values = array.as_vec<uint8_t>();
    std::vector<float> result(values.size());
    std::transform(values.begin(), values.end(), result.begin(), [](uint8_t v) {
      return static_cast<float>(v);
    });
    return result;
  }
  throw std::runtime_error("Unsupported word size for npz key " + key);
}

std::vector<float> replicate_for_batch(const std::vector<float> & single, int batch_size)
{
  std::vector<float> result;
  result.reserve(single.size() * static_cast<size_t>(batch_size));
  for (int i = 0; i < batch_size; ++i) {
    result.insert(result.end(), single.begin(), single.end());
  }
  return result;
}

std::vector<float> as_batched(
  const cnpy::npz_t & npz, const std::string & key, size_t single_size, int batch_size)
{
  const auto values = load_array_as_float(npz, key);
  if (values.size() == single_size) {
    return replicate_for_batch(values, batch_size);
  }
  if (values.size() == single_size * static_cast<size_t>(batch_size)) {
    return values;
  }
  throw std::runtime_error(
    "Unexpected size for " + key + ": got " + std::to_string(values.size()) + ", expected " +
    std::to_string(single_size) + " or " +
    std::to_string(single_size * static_cast<size_t>(batch_size)));
}

std::vector<float> heading_to_cos_sin(const std::vector<float> & values)
{
  if (values.size() % 3 != 0) {
    throw std::runtime_error("Heading input size must be divisible by 3.");
  }
  std::vector<float> result;
  result.reserve(values.size() / 3 * 4);
  for (size_t i = 0; i < values.size(); i += 3) {
    result.push_back(values[i]);
    result.push_back(values[i + 1]);
    result.push_back(std::cos(values[i + 2]));
    result.push_back(std::sin(values[i + 2]));
  }
  return result;
}

std::vector<float> load_pose_tensor(
  const cnpy::npz_t & npz, const std::string & key, size_t pose_count, int batch_size)
{
  const auto values = load_array_as_float(npz, key);
  if (values.size() == pose_count * POSE_DIM) {
    return replicate_for_batch(values, batch_size);
  }
  if (values.size() == pose_count * 3) {
    return replicate_for_batch(heading_to_cos_sin(values), batch_size);
  }
  if (values.size() == pose_count * POSE_DIM * static_cast<size_t>(batch_size)) {
    return values;
  }
  if (values.size() == pose_count * 3 * static_cast<size_t>(batch_size)) {
    return heading_to_cos_sin(values);
  }
  throw std::runtime_error("Unexpected pose tensor size for " + key);
}

std::vector<float> create_sampled_trajectories(double temperature, int batch_size)
{
  std::mt19937 gen(42);
  std::normal_distribution<float> dist(0.0f, 1.0f);
  std::vector<float> values(
    static_cast<size_t>(batch_size) * MAX_NUM_AGENTS * (OUTPUT_T + 1) * POSE_DIM);
  for (auto & value : values) {
    value = dist(gen) * static_cast<float>(temperature);
  }
  return values;
}

InputDataMap load_input_data(const Options & options)
{
  const auto npz = cnpy::npz_load(options.npz_path);
  InputDataMap input;

  const auto maybe_sampled = npz.find("sampled_trajectories");
  if (maybe_sampled != npz.end()) {
    input["sampled_trajectories"] = as_batched(
      npz, "sampled_trajectories", MAX_NUM_AGENTS * (OUTPUT_T + 1) * POSE_DIM, options.batch_size);
  } else {
    input["sampled_trajectories"] =
      create_sampled_trajectories(options.temperature, options.batch_size);
  }

  input["ego_agent_past"] =
    load_pose_tensor(npz, "ego_agent_past", INPUT_T + 1, options.batch_size);
  input["ego_current_state"] = as_batched(npz, "ego_current_state", 10, options.batch_size);
  input["neighbor_agents_past"] = as_batched(
    npz, "neighbor_agents_past", MAX_NUM_NEIGHBORS * (INPUT_T + 1) * 11, options.batch_size);
  input["static_objects"] = as_batched(npz, "static_objects", 5 * 10, options.batch_size);
  input["lanes"] =
    as_batched(npz, "lanes", num_elements_without_batch(LANES_SHAPE), options.batch_size);
  input["lanes_speed_limit"] = as_batched(
    npz, "lanes_speed_limit", num_elements_without_batch(LANES_SPEED_LIMIT_SHAPE),
    options.batch_size);
  input["route_lanes"] = as_batched(
    npz, "route_lanes", num_elements_without_batch(ROUTE_LANES_SHAPE), options.batch_size);
  input["route_lanes_speed_limit"] = as_batched(
    npz, "route_lanes_speed_limit", num_elements_without_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE),
    options.batch_size);
  input["polygons"] =
    as_batched(npz, "polygons", num_elements_without_batch(POLYGONS_SHAPE), options.batch_size);
  input["line_strings"] = as_batched(
    npz, "line_strings", num_elements_without_batch(LINE_STRINGS_SHAPE), options.batch_size);
  input["goal_pose"] = load_pose_tensor(npz, "goal_pose", 1, options.batch_size);

  const auto ego_shape_it = npz.find("ego_shape");
  input["ego_shape"] = ego_shape_it == npz.end()
                         ? replicate_for_batch({2.75f, 4.34f, 1.70f}, options.batch_size)
                         : as_batched(npz, "ego_shape", 3, options.batch_size);
  input["turn_indicators"] = as_batched(npz, "turn_indicators", INPUT_T + 1, options.batch_size);

  const auto normalization_map =
    autoware::diffusion_planner::utils::load_normalization_stats(options.args_path);
  autoware::diffusion_planner::preprocess::normalize_input_data(input, normalization_map);
  return input;
}

size_t prediction_index(size_t step, int batch, int agent, int t, int d, int batch_size)
{
  return (((step * static_cast<size_t>(batch_size) + batch) * MAX_NUM_AGENTS + agent) * OUTPUT_T +
          t) *
           POSE_DIM +
         d;
}

std::pair<std::vector<double>, std::vector<double>> extract_ego_trajectory(
  const std::vector<float> & values, size_t step, int batch_size)
{
  std::vector<double> x;
  std::vector<double> y;
  x.reserve(OUTPUT_T);
  y.reserve(OUTPUT_T);
  for (int t = 0; t < OUTPUT_T; ++t) {
    x.push_back(values[prediction_index(step, 0, 0, t, 0, batch_size)]);
    y.push_back(values[prediction_index(step, 0, 0, t, 1, batch_size)]);
  }
  return {x, y};
}

std::pair<std::vector<double>, std::vector<double>> bounds_from_predictions(
  const std::vector<float> & values, size_t num_steps, int batch_size)
{
  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(num_steps * OUTPUT_T);
  ys.reserve(num_steps * OUTPUT_T);
  for (size_t step = 0; step < num_steps; ++step) {
    const auto [x, y] = extract_ego_trajectory(values, step, batch_size);
    xs.insert(xs.end(), x.begin(), x.end());
    ys.insert(ys.end(), y.begin(), y.end());
  }
  return {xs, ys};
}

void apply_equal_limits(
  const autoware::pyplot::Axes & ax, const std::vector<double> & xs, const std::vector<double> & ys)
{
  if (xs.empty() || ys.empty()) {
    return;
  }
  const auto [min_x, max_x] = std::minmax_element(xs.begin(), xs.end());
  const auto [min_y, max_y] = std::minmax_element(ys.begin(), ys.end());
  const double center_x = (*min_x + *max_x) * 0.5;
  const double center_y = (*min_y + *max_y) * 0.5;
  const double span = std::max(*max_x - *min_x, *max_y - *min_y) * 0.55 + 1.0;
  ax.set_xlim(Args(center_x - span, center_x + span));
  ax.set_ylim(Args(center_y - span, center_y + span));
}

void plot_result(
  const autoware::diffusion_planner::MultiStepInference::InferenceResult & result,
  const Options & options)
{
  pybind11::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();

  const size_t single_prediction_size =
    static_cast<size_t>(options.batch_size) * MAX_NUM_AGENTS * OUTPUT_T * POSE_DIM;
  const size_t num_steps = result.denoising_timesteps.size();

  if (!options.overlay) {
    const int cols = 4;
    const int rows = static_cast<int>((num_steps + cols - 1) / cols);
    const auto [all_x, all_y] =
      bounds_from_predictions(result.denoising_predictions, num_steps, options.batch_size);

    auto subplots =
      plt.subplots(rows, cols, Kwargs("figsize"_a = pybind11::make_tuple(4 * cols, 3.5 * rows)));
    auto & axes = std::get<1>(subplots);
    for (size_t step = 0; step < num_steps; ++step) {
      const auto & ax = axes.at(step);
      const auto [x, y] =
        extract_ego_trajectory(result.denoising_predictions, step, options.batch_size);
      ax.plot(Args(x, y), Kwargs("color"_a = "tab:blue", "linewidth"_a = 1.6));
      ax.scatter(
        Args(std::vector<double>{x.front()}, std::vector<double>{y.front()}),
        Kwargs("color"_a = "black", "s"_a = 12));
      ax.set_title(Args(
        "step " + std::to_string(step) +
        "  t=" + std::to_string(result.denoising_timesteps[step])));
      apply_equal_limits(ax, all_x, all_y);
      ax.set_aspect(Args("equal"));
      ax.grid();
    }

    if (!options.output_path.empty()) {
      plt.savefig(Args(options.output_path), Kwargs("dpi"_a = 160));
      std::cout << "Saved figure: " << options.output_path << "\n";
    } else {
      plt.show();
    }
    return;
  }

  plt.figure(Args(), Kwargs("figsize"_a = pybind11::make_tuple(9, 8)));
  for (size_t step = 0; step < num_steps; ++step) {
    const auto [x, y] =
      extract_ego_trajectory(result.denoising_predictions, step, options.batch_size);
    const double alpha =
      0.25 + 0.65 * static_cast<double>(step + 1) / static_cast<double>(num_steps);
    const std::string label = (step == 0 || step + 1 == num_steps)
                                ? ("denoise " + std::to_string(step) +
                                   " t=" + std::to_string(result.denoising_timesteps[step]))
                                : "";
    plt.plot(
      Args(x, y),
      Kwargs("color"_a = "tab:blue", "alpha"_a = alpha, "linewidth"_a = 1.0, "label"_a = label));
  }

  if (result.outputs) {
    const auto & final_prediction = result.outputs->first;
    if (final_prediction.size() == single_prediction_size) {
      std::vector<float> final_as_step;
      final_as_step.reserve(single_prediction_size);
      final_as_step.insert(final_as_step.end(), final_prediction.begin(), final_prediction.end());
      const auto [x, y] = extract_ego_trajectory(final_as_step, 0, options.batch_size);
      plt.plot(
        Args(x, y),
        Kwargs("color"_a = "black", "linewidth"_a = 2.5, "label"_a = "final prediction"));
      plt.scatter(
        Args(std::vector<double>{x.front()}, std::vector<double>{y.front()}),
        Kwargs("color"_a = "black", "s"_a = 30));
    }
  }

  plt.title(Args("DiffusionPlannerInference denoising trajectory"));
  plt.xlabel(Args("x [m]"));
  plt.ylabel(Args("y [m]"));
  plt.axis(Args("equal"));
  plt.grid();
  plt.legend();

  if (!options.output_path.empty()) {
    plt.savefig(Args(options.output_path), Kwargs("dpi"_a = 160));
    std::cout << "Saved figure: " << options.output_path << "\n";
  } else {
    plt.show();
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    auto input = load_input_data(options);

    autoware::diffusion_planner::MultiStepInference inference(
      options.encoder_path, options.decoder_path, options.turn_indicator_path, options.args_path,
      options.plugins_path, options.batch_size, options.steps);
    const auto result = inference.infer(input);
    if (!result.outputs) {
      std::cerr << "Inference failed: " << result.error_msg << "\n";
      return 1;
    }

    std::cout << "prediction size: " << result.outputs->first.size() << "\n";
    std::cout << "turn_indicator_logit size: " << result.outputs->second.size() << "\n";
    std::cout << "denoising steps: " << result.denoising_timesteps.size() << "\n";
    std::cout << "denoising prediction size: " << result.denoising_predictions.size() << "\n";

    plot_result(result, options);
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    return 1;
  }

  return 0;
}
