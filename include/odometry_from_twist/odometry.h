#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace odometry {

Eigen::Isometry3d step(const Eigen::Vector3d &linear,
                       const Eigen::Vector3d &angular, double dt);

Eigen::Matrix<double, 6, 6>
covariance_step(const Eigen::Isometry3d step,
                const Eigen::Matrix<double, 6, 6> &covariance);

} // namespace odometry