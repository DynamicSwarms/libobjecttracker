#pragma once

#include <algorithm>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "libobjecttracker/object.hpp"
#include "libobjecttracker/tracking_event.hpp"

namespace libobjecttracker {

class TrackerBase
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  TrackerBase(
    const std::vector<DynamicsConfiguration>& dynamics_configurations,
    const std::vector<MarkerConfiguration>& marker_configurations,
    const std::function<void(const std::string&)>& log_warning,
    const TrackingEventCallback& event_callback)
    : m_dynamics_configurations(dynamics_configurations),
      m_marker_configurations(marker_configurations),
      m_log_warning(log_warning),
      m_event_callback(event_callback)
  {
  }

  virtual ~TrackerBase() = default;

  TrackerBase(const TrackerBase&) = delete;
  TrackerBase& operator=(const TrackerBase&) = delete;

  bool removeObject(const std::string& name)
  {
    const auto object = findObject(name);
    if (object == m_objects.end()) {
      return false;
    }
    m_objects.erase(object);
    return true;
  }

  bool removeAllObjects()
  {
    const bool removed = !m_objects.empty();
    m_objects.clear();
    return removed;
  }

  const std::vector<Object>& objects() const
  {
    return m_objects;
  }

  virtual void update(
    Cloud::ConstPtr markers) = 0;

protected:
  void addValidatedObject(
    size_t marker_configuration_idx,
    size_t dynamics_configuration_idx,
    const Eigen::Affine3f& initial_transformation,
    const std::string& name,
    double max_initial_deviation,
    double initialization_timeout)
  {
    removeObject(name);
    m_objects.emplace_back(
      marker_configuration_idx, dynamics_configuration_idx,
      initial_transformation, name, max_initial_deviation,
      initialization_timeout, m_log_warning);
  }

  void writeBack(
    Object& object,
    const Eigen::Affine3f& transform,
    bool orientation_valid) const
  {
    const bool acquired = !object.m_initialized;
    object.m_transformation = transform;
    object.m_time_since_last_valid_update = TrackerDuration::zero();
    object.m_initialized = true;
    object.m_valid = true;
    object.m_orientation_valid = orientation_valid;
    if (acquired && m_event_callback) {
      m_event_callback(TrackingEvent::acquired, object);
    }
  }

  void logWarning(const Object& object, const std::string& reason) const
  {
    object.logWarning(reason);
  }

  void logWarning(const std::string& message) const
  {
    if (m_log_warning) {
      m_log_warning(message);
    }
  }

  std::vector<Object>::iterator findObject(const std::string& name)
  {
    return std::find_if(
      m_objects.begin(), m_objects.end(),
      [&](const Object& object) { return object.name() == name; });
  }

  const std::vector<DynamicsConfiguration>& m_dynamics_configurations;
  const std::vector<MarkerConfiguration>& m_marker_configurations;
  const std::function<void(const std::string&)>& m_log_warning;
  const TrackingEventCallback& m_event_callback;
  std::vector<Object> m_objects;

  friend ObjectTracker;
};

}  // namespace libobjecttracker
