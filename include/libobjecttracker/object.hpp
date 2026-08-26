#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <limits>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace libobjecttracker {

struct DynamicsConfiguration
{
  double maxXVelocity;
  double maxYVelocity;
  double maxZVelocity;
  double maxPitchRate;
  double maxRollRate;
  double maxYawRate;
  double maxRoll;
  double maxPitch;
  double maxFitnessScore;
};

using MarkerConfiguration = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using TrackerDuration = std::chrono::duration<double>;
using LogWarningCallback = std::function<void(const std::string&)>;
constexpr double tracking_timeout_seconds = 0.5;

class ObjectTracker;
class TrackerBase;
class SingleMarkerTracker;
class MultiMarkerTracker;
class PointCloudDebugger;

class Object
{
public:
  Object(
    size_t marker_configuration_idx,
    size_t dynamics_configuration_idx,
    const Eigen::Affine3f& initial_transformation,
    const std::string& name,
    double max_initial_deviation,
    double initialization_timeout,
    const LogWarningCallback& log_warning)
    : m_marker_configuration_idx(marker_configuration_idx),
      m_dynamics_configuration_idx(dynamics_configuration_idx),
      m_transformation(initial_transformation),
      m_initial_transformation(initial_transformation),
      m_time_since_registration(TrackerDuration::zero()),
      m_time_since_last_valid_update(TrackerDuration::zero()),
      m_time_since_last_warning(TrackerDuration::zero()),
      m_initialized(false),
      m_valid(false),
      m_orientation_valid(false),
      m_name(name),
      m_max_initial_deviation(max_initial_deviation),
      m_initialization_timeout(initialization_timeout),
      m_log_warning(log_warning)
  {
  }

  const Eigen::Affine3f& transformation() const { return m_transformation; }
  Eigen::Vector3f center() const { return m_transformation.translation(); }
  const Eigen::Affine3f& initialTransformation() const
  {
    return m_initial_transformation;
  }
  Eigen::Vector3f initialCenter() const
  {
    return m_initial_transformation.translation();
  }
  bool initialized() const { return m_initialized; }
  bool lastTransformationValid() const { return m_valid; }
  bool orientationValid() const { return m_orientation_valid; }
  const std::string& name() const { return m_name; }

  TrackerDuration timeSinceLastValidUpdate() const
  {
    if (!m_initialized) {
      return TrackerDuration::max();
    }
    return m_time_since_last_valid_update;
  }

  TrackerDuration timeSinceRegistration() const
  {
    return m_time_since_registration;
  }

private:
  void advance(TrackerDuration dt)
  {
    m_time_since_registration += dt;
    m_time_since_last_warning += dt;
    if (m_initialized) {
      m_time_since_last_valid_update += dt;
    }
  }

  void noValidSolution() const
  {
    if (!m_initialized) {
      return;
    }
    constexpr TrackerDuration warning_delay{0.1};
    constexpr TrackerDuration warning_interval{0.05};
    if (m_time_since_last_valid_update > warning_delay &&
      m_time_since_last_warning >= warning_interval) {
      logWarning("No valid assignment for " +
        std::to_string(m_time_since_last_valid_update.count()) + " s");
      m_time_since_last_warning = TrackerDuration::zero();
    }
  }

  void logWarning(const std::string& reason) const
  {
    if (m_log_warning) {
      m_log_warning("Object '" + m_name + "': " + reason);
    }
  }

  size_t m_marker_configuration_idx;
  size_t m_dynamics_configuration_idx;
  Eigen::Affine3f m_transformation;
  Eigen::Affine3f m_initial_transformation;
  TrackerDuration m_time_since_registration;
  TrackerDuration m_time_since_last_valid_update;
  mutable TrackerDuration m_time_since_last_warning;
  bool m_initialized;
  bool m_valid;
  bool m_orientation_valid;
  std::string m_name;
  double m_max_initial_deviation;
  double m_initialization_timeout;
  LogWarningCallback m_log_warning;
  friend ObjectTracker;
  friend TrackerBase;
  friend SingleMarkerTracker;
  friend MultiMarkerTracker;
  friend PointCloudDebugger;
};

}  // namespace libobjecttracker
