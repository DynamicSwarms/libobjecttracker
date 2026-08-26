#pragma once

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libobjecttracker/multi_marker_tracker.hpp"
#include "libobjecttracker/object.hpp"
#include "libobjecttracker/single_marker_tracker.hpp"

namespace libobjecttracker {

class ObjectTracker
{
public:
  static constexpr double tracking_timeout = tracking_timeout_seconds;

  ObjectTracker(
    const std::vector<DynamicsConfiguration>& dynamics_configurations,
    const std::vector<MarkerConfiguration>& marker_configurations)
    : m_dynamics_configurations(dynamics_configurations),
      m_marker_configurations(marker_configurations),
      m_single_tracker(
        m_dynamics_configurations, m_marker_configurations, m_log_warning,
        m_event_callback),
      m_multi_tracker(
        m_dynamics_configurations, m_marker_configurations, m_log_warning,
        m_event_callback)
  {
  }

  bool addObject(
    size_t marker_configuration_idx,
    size_t dynamics_configuration_idx,
    const Eigen::Affine3f& initial_transformation,
    const std::string& name,
    double max_initial_deviation,
    double initialization_timeout)
  {
    if (!validateObject(
        marker_configuration_idx, dynamics_configuration_idx,
        initial_transformation, name, max_initial_deviation,
        initialization_timeout)) {
      return false;
    }
    const auto& markers = m_marker_configurations[marker_configuration_idx];

    TrackerBase* target_tracker = markers->size() == 1
      ? static_cast<TrackerBase*>(&m_single_tracker)
      : static_cast<TrackerBase*>(&m_multi_tracker);
    target_tracker->addValidatedObject(
      marker_configuration_idx, dynamics_configuration_idx,
      initial_transformation, name, max_initial_deviation,
      initialization_timeout);

    if (target_tracker == &m_single_tracker) {
      m_multi_tracker.removeObject(name);
    } else {
      m_single_tracker.removeObject(name);
    }
    return true;
  }

  bool removeObject(const std::string& name)
  {
    const bool single_removed = m_single_tracker.removeObject(name);
    const bool multi_removed = m_multi_tracker.removeObject(name);
    return single_removed || multi_removed;
  }

  size_t removeStaleObjects()
  {
    std::vector<std::pair<std::string, TrackingEvent>> stale_objects;
    for (const auto* object : objects()) {
      if (!object->m_initialized &&
        object->timeSinceRegistration() >=
          TrackerDuration(object->m_initialization_timeout)) {
        stale_objects.emplace_back(
          object->name(), TrackingEvent::initialization_timed_out);
      } else if (object->m_initialized && !trackingValid(*object)) {
        stale_objects.emplace_back(object->name(), TrackingEvent::lost);
      }
    }

    for (const auto& stale_object : stale_objects) {
      for (const auto* object : objects()) {
        if (object->name() == stale_object.first && m_event_callback) {
          m_event_callback(stale_object.second, *object);
          break;
        }
      }
      removeObject(stale_object.first);
    }
    return stale_objects.size();
  }

  bool removeAllObjects()
  {
    const bool single_removed = m_single_tracker.removeAllObjects();
    const bool multi_removed = m_multi_tracker.removeAllObjects();
    return single_removed || multi_removed;
  }

  bool trackingValid(const Object& object) const
  {
    return object.m_initialized &&
      object.timeSinceLastValidUpdate() < TrackerDuration(tracking_timeout);
  }

  void update(
    TrackerDuration dt,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud)
  {
    if (dt < TrackerDuration::zero()) {
      throw std::invalid_argument("ObjectTracker update duration cannot be negative");
    }
    for (auto& object : m_single_tracker.m_objects) {
      object.advance(dt);
    }
    for (auto& object : m_multi_tracker.m_objects) {
      object.advance(dt);
    }
    m_single_tracker.update(point_cloud);
    m_multi_tracker.update(point_cloud);
  }

  std::vector<const Object*> objects() const
  {
    std::vector<const Object*> result;
    const auto& single_objects = m_single_tracker.objects();
    const auto& multi_objects = m_multi_tracker.objects();
    result.reserve(single_objects.size() + multi_objects.size());
    for (const auto& object : single_objects) {
      result.push_back(&object);
    }
    for (const auto& object : multi_objects) {
      result.push_back(&object);
    }
    return result;
  }

  const std::vector<Object>& singleMarkerObjects() const
  {
    return m_single_tracker.objects();
  }

  const std::vector<Object>& multiMarkerObjects() const
  {
    return m_multi_tracker.objects();
  }

  void setLogWarningCallback(
    std::function<void(const std::string&)> log_warning)
  {
    m_log_warning = std::move(log_warning);
  }

  void setEventCallback(TrackingEventCallback event_callback)
  {
    m_event_callback = std::move(event_callback);
  }

private:
  bool validateObject(
    size_t marker_configuration_idx,
    size_t dynamics_configuration_idx,
    const Eigen::Affine3f& initial_transformation,
    const std::string& name,
    double max_initial_deviation,
    double initialization_timeout) const
  {
    if (name.empty()) {
      logWarning("Cannot add object: name is empty");
      return false;
    }
    if (marker_configuration_idx >= m_marker_configurations.size()) {
      logWarning("Cannot add object '" + name +
        "': marker configuration index is out of range");
      return false;
    }
    if (dynamics_configuration_idx >= m_dynamics_configurations.size()) {
      logWarning("Cannot add object '" + name +
        "': dynamics configuration index is out of range");
      return false;
    }
    const auto& markers = m_marker_configurations[marker_configuration_idx];
    if (!markers || markers->empty()) {
      logWarning("Cannot add object '" + name +
        "': marker configuration is empty");
      return false;
    }
    if (!initial_transformation.matrix().allFinite()) {
      logWarning("Cannot add object '" + name +
        "': initial transformation is invalid");
      return false;
    }
    if (!std::isfinite(max_initial_deviation) || max_initial_deviation < 0.0) {
      logWarning("Cannot add object '" + name +
        "': max initial deviation is invalid");
      return false;
    }
    if (!std::isfinite(initialization_timeout) || initialization_timeout <= 0.0) {
      logWarning("Cannot add object '" + name +
        "': initialization timeout is invalid");
      return false;
    }
    return true;
  }

  void logWarning(const std::string& message) const
  {
    if (m_log_warning) {
      m_log_warning(message);
    }
  }

  std::vector<DynamicsConfiguration> m_dynamics_configurations;
  std::vector<MarkerConfiguration> m_marker_configurations;
  std::function<void(const std::string&)> m_log_warning;
  TrackingEventCallback m_event_callback;
  SingleMarkerTracker m_single_tracker;
  MultiMarkerTracker m_multi_tracker;
};

}  // namespace libobjecttracker
