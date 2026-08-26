#pragma once

#include <Eigen/Core>

#include <pcl/point_types.h>

namespace libobjecttracker {

inline Eigen::Vector3f to_eigen(const pcl::PointXYZ& point)
{
  return {point.x, point.y, point.z};
}

inline pcl::PointXYZ to_pcl(const Eigen::Vector3f& vector)
{
  return {vector.x(), vector.y(), vector.z()};
}

}  // namespace libobjecttracker
