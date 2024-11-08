#include <odometry_from_twist/odometry.h>

namespace odometry {

Eigen::Isometry3d step(const Eigen::Vector3d &linear,
                       const Eigen::Vector3d &angular, double dt) {
  return Eigen::Translation3d(dt * linear) *
         Eigen::AngleAxisd(dt * angular.x(), Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(dt * angular.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(dt * angular.z(), Eigen::Vector3d::UnitZ());
}

Eigen::Matrix<double, 6, 6>
covariance_step(const Eigen::Isometry3d step,
                const Eigen::Matrix<double, 6, 6> &covariance) {}

} // namespace odometry