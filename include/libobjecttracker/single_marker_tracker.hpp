#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

#include "../../src/assignment.hpp"
#include "libobjecttracker/conversions.hpp"
#include "libobjecttracker/tracker_base.hpp"

namespace libobjecttracker {

class SingleMarkerTracker : public TrackerBase
{
public:
  SingleMarkerTracker(
    const std::vector<DynamicsConfiguration>& dynamics_configurations,
    const std::vector<MarkerConfiguration>& marker_configurations,
    const std::function<void(const std::string&)>& log_warning,
    const TrackingEventCallback& event_callback)
    : TrackerBase(
        dynamics_configurations, marker_configurations, log_warning,
        event_callback)
  {
  }

  void update(
    Cloud::ConstPtr markers) override
  {
    for (auto& object : m_objects) {
      object.m_valid = false;
    }
    if (markers->empty() || m_objects.empty()) {
      for (const auto& object : m_objects) {
        object.noValidSolution();
      }
      return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(markers);
    libMultiRobotPlanning::Assignment<size_t, size_t> assignment;

    for (size_t object_idx = 0; object_idx < m_objects.size(); ++object_idx) {
      addCandidates(object_idx, markers, kdtree, assignment);
    }

    std::map<size_t, size_t> solution;
    assignment.solve(solution);
    for (size_t object_idx = 0; object_idx < m_objects.size(); ++object_idx) {
      const auto match = solution.find(object_idx);
      if (match != solution.end()) {
        writeMarkerBack(
          m_objects[object_idx], (*markers)[match->second]);
      } else {
        m_objects[object_idx].noValidSolution();
      }
    }
  }

private:
  using Assignment = libMultiRobotPlanning::Assignment<size_t, size_t>;

  void addCandidates(
    size_t object_idx,
    Cloud::ConstPtr markers,
    pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
    Assignment& assignment) const
  {
    const auto& object = m_objects[object_idx];
    const Eigen::Vector3f reference = object.m_initialized
      ? object.center() : object.initialCenter();
    std::vector<int> neighbor_indices(5);
    std::vector<float> neighbor_distances(5);
    const int found = kdtree.nearestKSearch(
      to_pcl(reference), neighbor_indices.size(), neighbor_indices,
      neighbor_distances);

    for (int neighbor = 0; neighbor < found; ++neighbor) {
      const size_t marker_idx = neighbor_indices[neighbor];
      const Eigen::Vector3f marker = to_eigen((*markers)[marker_idx]);
      if (!validCandidate(object, marker)) {
        continue;
      }
      const long cost = static_cast<long>((marker - reference).norm() * 1000.0);
      assignment.setCost(object_idx, marker_idx, cost);
    }
  }

  bool validCandidate(
    const Object& object,
    const Eigen::Vector3f& marker) const
  {
    const Eigen::Vector3f offset = to_eigen(
      (*m_marker_configurations[object.m_marker_configuration_idx])[0]);
    if (!object.m_initialized) {
      const Eigen::Vector3f expected_marker =
        object.m_initial_transformation * offset;
      return (marker - expected_marker).norm() <= object.m_max_initial_deviation;
    }

    const double dt = object.m_time_since_last_valid_update.count();
    if (dt <= 0.0 || dt > tracking_timeout_seconds) {
      return false;
    }
    const Eigen::Vector3f position =
      marker - object.m_transformation.linear() * offset;
    const Eigen::Vector3f velocity = (position - object.center()) / dt;
    const auto& dynamics =
      m_dynamics_configurations[object.m_dynamics_configuration_idx];
    return std::abs(velocity.x()) < dynamics.maxXVelocity &&
      std::abs(velocity.y()) < dynamics.maxYVelocity &&
      std::abs(velocity.z()) < dynamics.maxZVelocity;
  }

  void writeMarkerBack(
    Object& object,
    const pcl::PointXYZ& marker) const
  {
    const Eigen::Vector3f offset = to_eigen(
      (*m_marker_configurations[object.m_marker_configuration_idx])[0]);
    Eigen::Affine3f transform = object.m_transformation;
    transform.translation() =
      to_eigen(marker) - transform.linear() * offset;

    writeBack(object, transform, false);
  }
};

}  // namespace libobjecttracker
