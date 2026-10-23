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

#include "autoware/ml_planner/utils/marker_utils.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <rclcpp/duration.hpp>

#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/point.hpp>  // Include the header for Point

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace autoware::ml_planner::utils
{
using geometry_msgs::msg::Point;

namespace
{
// Helper function to create a basic marker with common properties
Marker create_base_marker(
  const Time & stamp, const std::string & frame_id, const std::string & ns, int id, int type,
  const ColorRGBA & color, const rclcpp::Duration & lifetime, double scale_x)
{
  Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale_x;
  marker.color = color;
  marker.lifetime = lifetime;
  return marker;
}

// Helper to transform an ego-frame point to the target frame
Eigen::Vector4d transform_point(
  const xt::xarray<float> & tensor, size_t l, size_t p, int64_t offset,
  const Eigen::Matrix4d & transform)
{
  const Eigen::Vector4d point(
    tensor(l, p, offset + 0), tensor(l, p, offset + 1), tensor(l, p, offset + 2), 1.0);
  return transform * point;
}

// Helper to add point to marker
void add_point_to_marker(Marker & marker, double x, double y, double z)
{
  Point pt;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  marker.points.push_back(pt);
}

}  // anonymous namespace

MarkerArray create_map_polyline_marker(
  const Eigen::Matrix4d & transform_ego_to_map, const xt::xarray<float> & polyline_tensor,
  const Time & stamp, const rclcpp::Duration & lifetime, const std::array<float, 4> & colors,
  const std::string & marker_namespace, const std::string & frame_id)
{
  MarkerArray marker_array;
  if (
    polyline_tensor.dimension() != 3 ||
    polyline_tensor.shape(2) != static_cast<size_t>(POLYLINE_3D_POINT_DIM)) {
    return marker_array;
  }
  const size_t num_polylines = polyline_tensor.shape(0);
  const size_t num_points = polyline_tensor.shape(1);
  constexpr float near_zero_threshold = 1e-2f;
  ColorRGBA color;
  color.r = colors[0];
  color.g = colors[1];
  color.b = colors[2];
  color.a = colors[3];

  for (size_t l = 0; l < num_polylines; ++l) {
    Marker marker = create_base_marker(
      stamp, frame_id, marker_namespace, static_cast<int>(l), Marker::LINE_STRIP, color, lifetime,
      0.2);

    for (size_t p = 0; p < num_points; ++p) {
      // Skip near-zero points (padding)
      if (std::hypot(polyline_tensor(l, p, 0), polyline_tensor(l, p, 1)) < near_zero_threshold) {
        continue;
      }

      const Eigen::Vector4d pt_map =
        transform_point(polyline_tensor, l, p, 0, transform_ego_to_map);
      add_point_to_marker(marker, pt_map.x(), pt_map.y(), pt_map.z() + 0.1);
    }

    if (marker.points.empty()) {
      continue;
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

MarkerArray create_lane_marker(
  const Eigen::Matrix4d & transform_ego_to_map, const xt::xarray<float> & lane_tensor,
  const Time & stamp, const rclcpp::Duration & lifetime, const std::array<float, 4> colors,
  const std::string & frame_id)
{
  MarkerArray marker_array;
  if (
    lane_tensor.dimension() != 3 ||
    lane_tensor.shape(2) != static_cast<size_t>(LANE_3D_POINT_DIM)) {
    return marker_array;
  }
  const size_t num_segments = lane_tensor.shape(0);
  const size_t num_points = lane_tensor.shape(1);
  int64_t segment_count = 0;
  constexpr double near_zero_threshold = 1e-2;
  // Setup colors
  ColorRGBA lane_color;
  lane_color.r = colors[0];
  lane_color.g = colors[1];
  lane_color.b = colors[2];
  lane_color.a = colors[3];

  ColorRGBA bounds_color;
  bounds_color.r = 0.9f;
  bounds_color.g = 0.65f;
  bounds_color.b = 0.0f;
  bounds_color.a = 0.8f;

  for (size_t l = 0; l < num_segments; ++l) {
    // Create markers for this segment
    Marker marker_centerline = create_base_marker(
      stamp, frame_id, "lane", static_cast<int>(l), Marker::LINE_STRIP, lane_color, lifetime, 0.3);

    Marker marker_left_bound = create_base_marker(
      stamp, frame_id, "lane_lb", static_cast<int>(l), Marker::LINE_STRIP, bounds_color, lifetime,
      0.3);

    Marker marker_right_bound = create_base_marker(
      stamp, frame_id, "lane_rb", static_cast<int>(l), Marker::LINE_STRIP, bounds_color, lifetime,
      0.3);

    // Sphere marker with alternating colors
    ColorRGBA sphere_color;
    sphere_color.r = segment_count % 2 == 0 ? 0.1f : 0.9f;
    sphere_color.g = segment_count % 2 == 0 ? 0.9f : 0.1f;
    sphere_color.b = 0.9f;
    sphere_color.a = 0.8f;

    Marker marker_sphere = create_base_marker(
      stamp, frame_id, "sphere", static_cast<int>(l), Marker::SPHERE_LIST, sphere_color, lifetime,
      0.5);
    marker_sphere.scale.y = 0.5;
    marker_sphere.scale.z = 0.5;

    // Process points for this segment
    for (size_t p = 0; p < num_points; ++p) {
      // Skip near-zero points (padding)
      const double norm =
        std::hypot(lane_tensor(l, p, LANE_3D_CENTER + 0), lane_tensor(l, p, LANE_3D_CENTER + 1));
      if (norm < near_zero_threshold) {
        continue;
      }

      // Transform points from ego to map frame
      const Eigen::Vector4d center =
        transform_point(lane_tensor, l, p, LANE_3D_CENTER, transform_ego_to_map);
      const Eigen::Vector4d left =
        transform_point(lane_tensor, l, p, LANE_3D_LB, transform_ego_to_map);
      const Eigen::Vector4d right =
        transform_point(lane_tensor, l, p, LANE_3D_RB, transform_ego_to_map);

      // Add points to respective markers
      add_point_to_marker(marker_centerline, center.x(), center.y(), center.z() + 0.1);
      add_point_to_marker(marker_left_bound, left.x(), left.y(), left.z() + 0.1);
      add_point_to_marker(marker_right_bound, right.x(), right.y(), right.z() + 0.1);
      add_point_to_marker(marker_sphere, center.x(), center.y(), center.z() + 0.2);
    }

    // Skip empty segments
    if (marker_centerline.points.empty()) {
      continue;
    }
    ++segment_count;

    marker_array.markers.push_back(marker_sphere);
    marker_array.markers.push_back(marker_centerline);
    marker_array.markers.push_back(marker_left_bound);
    marker_array.markers.push_back(marker_right_bound);
  }

  return marker_array;
}

}  // namespace autoware::ml_planner::utils
