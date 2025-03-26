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

#include "autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp"

#include "autoware/behavior_path_bidirectional_traffic_module/bidirectional_lanelets.hpp"
#include "autoware/trajectory/utils/closest.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

autoware_perception_msgs::msg::PredictedObjects extract_car_objects(
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects)
{
  auto is_car_object = [](const autoware_perception_msgs::msg::PredictedObject & object) {
    return !std::all_of(
      object.classification.begin(), object.classification.end(), [](const auto & c) {
        return c.label != autoware_perception_msgs::msg::ObjectClassification::CAR;
      });
  };
  autoware_perception_msgs::msg::PredictedObjects car_objects;
  for (const auto & object : predicted_objects.objects) {
    if (is_car_object(object)) {
      car_objects.objects.push_back(object);
    }
  }
  return car_objects;
}

size_t get_key(const std::array<unsigned char, 16> & uuid)
{
  return std::accumulate(
    uuid.begin(), uuid.end(), static_cast<std::size_t>(0),
    [](std::size_t seed, unsigned char byte) {
      return seed ^ (static_cast<std::size_t>(byte) + 0x9e3779b9 + (seed << 6U) + (seed >> 2U));
    });
}

OncomingCars::OncomingCars(
  ConnectedBidirectionalLanelets::SharedConstPtr bidirectional_lanelets,
  const double & time_to_include_in_oncoming_car,
  const std::function<void(std::string_view)> & logger)
: bidirectional_lanelets_(std::move(bidirectional_lanelets)),
  time_to_include_in_oncoming_car_(time_to_include_in_oncoming_car),
  logger_(logger)
{
}

std::optional<CarObject> find(const std::vector<CarObject> & oncoming_cars, const size_t & key)
{
  auto it = std::find_if(oncoming_cars.begin(), oncoming_cars.end(), [key](const CarObject & car) {
    return get_key(car.get_uuid().uuid) == key;
  });

  if (it != oncoming_cars.end()) {
    return *it;
  }

  return std::nullopt;
}

std::optional<autoware_perception_msgs::msg::PredictedObject> find(
  const autoware_perception_msgs::msg::PredictedObjects & car_objects, const size_t & key)
{
  auto it = std::find_if(
    car_objects.objects.begin(), car_objects.objects.end(),
    [key](const auto & object) { return get_key(object.object_id.uuid) == key; });

  if (it != car_objects.objects.end()) {
    return *it;
  }

  return std::nullopt;
}

template <typename T>
std::optional<std::pair<size_t, T>> find(const std::map<size_t, T> & map, const size_t & key)
{
  auto it = map.find(key);
  if (it != map.end()) {
    return *it;
  }

  return std::nullopt;
}

void erase(std::vector<CarObject> * oncoming_cars, const CarObject & car)
{
  oncoming_cars->erase(
    std::remove_if(
      oncoming_cars->begin(), oncoming_cars->end(),
      [&car](const CarObject & oncoming_car) {
        return get_key(oncoming_car.get_uuid().uuid) == get_key(car.get_uuid().uuid);
      }),
    oncoming_cars->end());
}

double distance_on_lane(
  const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2,
  const ConnectedBidirectionalLanelets::SharedConstPtr & bidirectional_lanelets)
{
  auto center_line = bidirectional_lanelets->get_center_line();
  double distance = autoware::trajectory::closest(center_line, pose2) -
                    autoware::trajectory::closest(center_line, pose1);
  return distance;
}

bool PredictedObjectsComparator::operator()(
  const autoware_perception_msgs::msg::PredictedObject & lhs,
  const autoware_perception_msgs::msg::PredictedObject & rhs) const
{
  return std::lexicographical_compare(
    lhs.object_id.uuid.begin(), lhs.object_id.uuid.end(), rhs.object_id.uuid.begin(),
    rhs.object_id.uuid.end());
}

CarObject::CarObject(
  geometry_msgs::msg::Pose pose, const double & speed, unique_identifier_msgs::msg::UUID uuid,
  ConnectedBidirectionalLanelets::SharedConstPtr bidirectional_lanelets)
: pose_(pose),
  speed_(speed),
  uuid_(uuid),
  bidirectional_lanelets_(std::move(bidirectional_lanelets))
{
}

void OncomingCars::update(
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  const geometry_msgs::msg::Pose & ego_pose, const rclcpp::Time & time)
{
  autoware_perception_msgs::msg::PredictedObjects current_car_objects =
    extract_car_objects(predicted_objects);

  for (const auto & object : current_car_objects.objects) {
    size_t object_key = get_key(object.object_id.uuid);
    bool running_opposite_lane =
      bidirectional_lanelets_->get_opposite()->is_object_on_this_lane(object);
    bool find_in_candidates = find(oncoming_cars_candidates_, object_key).has_value();
    bool find_in_oncoming_cars = find(oncoming_cars_, object_key).has_value();

    if (running_opposite_lane && !find_in_candidates && !find_in_oncoming_cars) {
      if (
        distance_on_lane(
          ego_pose, object.kinematics.initial_pose_with_covariance.pose, bidirectional_lanelets_) >
        0.0) {
        oncoming_cars_candidates_[object_key] = time;
      }
    }
    if (running_opposite_lane && find_in_candidates && !find_in_oncoming_cars) {
      if (
        time - oncoming_cars_candidates_[object_key] >
        rclcpp::Duration::from_seconds(time_to_include_in_oncoming_car_)) {
        oncoming_cars_.emplace_back(
          object.kinematics.initial_pose_with_covariance.pose,
          object.kinematics.initial_twist_with_covariance.twist.linear.x, object.object_id,
          bidirectional_lanelets_);
        oncoming_cars_candidates_.erase(object_key);
        logger_(fmt::format("Added oncoming car: {:03X}", object_key));
      }
    }
  }
  for (const auto & [oncoming_cars_candidate, _] : oncoming_cars_candidates_) {
    auto car_object = find(current_car_objects, oncoming_cars_candidate);
    bool shold_remove_from_candidates =
      !car_object.has_value() ||
      !bidirectional_lanelets_->get_opposite()->is_object_on_this_lane(car_object.value());
    if (shold_remove_from_candidates) {
      oncoming_cars_candidates_.erase(oncoming_cars_candidate);
    }
  }

  for (auto & oncoming_car : oncoming_cars_) {
    auto car_object = find(current_car_objects, get_key(oncoming_car.get_uuid().uuid));

    bool should_remove_from_oncoming_car =
      !car_object.has_value() ||
      (!bidirectional_lanelets_->get_opposite()->is_object_on_this_lane(car_object.value()) &&
       !bidirectional_lanelets_->is_object_on_this_lane(car_object.value())) ||
      (distance_on_lane(ego_pose, oncoming_car.get_pose(), bidirectional_lanelets_) < 0.0);

    if (should_remove_from_oncoming_car) {
      erase(&oncoming_cars_, oncoming_car);
      logger_(fmt::format("Removed oncoming car: {:03X}", get_key(oncoming_car.get_uuid().uuid)));
    } else {
      oncoming_car.update(
        car_object.value().kinematics.initial_pose_with_covariance.pose,
        car_object.value().kinematics.initial_twist_with_covariance.twist.linear.x);
    }
  }
}

const std::vector<CarObject> & OncomingCars::get_oncoming_cars() const
{
  return oncoming_cars_;
}

std::optional<CarObject> OncomingCars::get_front_oncoming_car() const
{
  if (oncoming_cars_.empty()) {
    return std::nullopt;
  }

  double min_distance = std::numeric_limits<double>::max();
  std::optional<CarObject> front_oncoming_car;
  for (const auto & oncoming_car : oncoming_cars_) {
    auto center_line = bidirectional_lanelets_->get_center_line();
    double distance = autoware::trajectory::closest(center_line, oncoming_car.get_pose());
    if (distance < min_distance) {
      min_distance = distance;
      front_oncoming_car = oncoming_car;
    }
  }

  return front_oncoming_car;
}

}  // namespace autoware::behavior_path_planner
