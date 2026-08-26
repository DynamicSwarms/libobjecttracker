#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/print.h>

#include "libobjecttracker/tracker_base.hpp"
#include "libobjecttracker/conversions.hpp"

namespace libobjecttracker {

class MultiMarkerTracker : public TrackerBase
{
public:
  MultiMarkerTracker(
    const std::vector<DynamicsConfiguration>& dynamics_configurations,
    const std::vector<MarkerConfiguration>& marker_configurations,
    const std::function<void(const std::string&)>& log_warning,
    const TrackingEventCallback& event_callback)
    : TrackerBase(
        dynamics_configurations, marker_configurations, log_warning,
        event_callback)
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  }

  void update(
    Cloud::ConstPtr markers) override
  {
    for (auto& object : m_objects) {
      object.m_valid = false;
      Eigen::Affine3f transform;
      if (alignObject(object, markers, transform)) {
        writeBack(object, transform, true);
      } else {
        object.noValidSolution();
      }
    }
  }

private:
  using ICP = pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;

  static float angleDifference(float lhs, float rhs)
  {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
  }

  bool alignObject(
    const Object& object,
    Cloud::ConstPtr markers,
    Eigen::Affine3f& transform) const
  {
    const auto& model =
      m_marker_configurations[object.m_marker_configuration_idx];
    if (markers->size() < model->size()) {
      return false;
    }

    const Eigen::Affine3f& guess = object.m_initialized
      ? object.m_transformation : object.m_initial_transformation;

    Cloud::Ptr neighbors = nearestNeighbors(guess, model->size(), markers);
    if (!neighbors) {
      return false;
    }

    const auto& dynamics = m_dynamics_configurations[object.m_dynamics_configuration_idx];
    const double dt = object.m_initialized
      ? object.m_time_since_last_valid_update.count() : 0.0;
    if (object.m_initialized && (dt <= 0.0 || dt > tracking_timeout_seconds)) {
      return false;
    }

    const double max_velocity = std::max(
      dynamics.maxXVelocity, 
      std::max(dynamics.maxYVelocity, dynamics.maxZVelocity));
    
    const double max_correspondence_distance = object.m_initialized
      ? max_velocity * dt : object.m_max_initial_deviation;

    ICP icp;
    icp.setMaximumIterations(5);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setInputSource(model);
    icp.setInputTarget(markers);
    Cloud aligned;
    icp.align(aligned, guess.matrix());
    if (!icp.hasConverged()) {
      // logWarning(object, "ICP did not converge");
      return false;
    }

    transform = Eigen::Affine3f(icp.getFinalTransformation());
    return validMatch(object, transform, icp.getFitnessScore());
  }

  Cloud::Ptr nearestNeighbors(
    const Eigen::Affine3f& guess,
    size_t count,
    Cloud::ConstPtr markers) const
  {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(markers);
    std::vector<int> indices(count);
    std::vector<float> distances(count);
    const int found = kdtree.nearestKSearch(
      to_pcl(guess.translation()), count, indices, distances);
    if (found != static_cast<int>(count)) {
      return {};
    }

    Cloud::Ptr neighbors(new Cloud);
    neighbors->reserve(count);
    for (const int marker_idx : indices) {
      neighbors->push_back((*markers)[marker_idx]);
    }
    return neighbors;
  }

  bool validMatch(
    const Object& object,
    const Eigen::Affine3f& transform,
    double fitness) const
  {
    const auto& dynamics =
      m_dynamics_configurations[object.m_dynamics_configuration_idx];
    if (fitness >= dynamics.maxFitnessScore) {
      // logWarning(object, "ICP fitness exceeds configured maximum");
      return false;
    }
    if (!object.m_initialized) {
      return (transform.translation() - object.initialCenter()).norm() <=
        object.m_max_initial_deviation;
    }

    const double dt = object.m_time_since_last_valid_update.count();
    float x, y, z, roll, pitch, yaw;
    float old_x, old_y, old_z, old_roll, old_pitch, old_yaw;
    pcl::getTranslationAndEulerAngles(transform, x, y, z, roll, pitch, yaw);
    pcl::getTranslationAndEulerAngles(
      object.m_transformation, old_x, old_y, old_z, old_roll,
      old_pitch, old_yaw);
    return std::abs((x - old_x) / dt) < dynamics.maxXVelocity &&
      std::abs((y - old_y) / dt) < dynamics.maxYVelocity &&
      std::abs((z - old_z) / dt) < dynamics.maxZVelocity &&
      std::abs(angleDifference(roll, old_roll) / dt) < dynamics.maxRollRate &&
      std::abs(angleDifference(pitch, old_pitch) / dt) < dynamics.maxPitchRate &&
      std::abs(angleDifference(yaw, old_yaw) / dt) < dynamics.maxYawRate &&
      std::abs(roll) < dynamics.maxRoll &&
      std::abs(pitch) < dynamics.maxPitch;
  }

};

}  // namespace libobjecttracker
