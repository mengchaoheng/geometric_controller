// SO(3) utilities for the embedded Lu OMMPC implementation.
#ifndef LU_OMMPC__SO3_HPP_
#define LU_OMMPC__SO3_HPP_

#include <Eigen/Dense>

namespace lu_ommpc
{

Eigen::Matrix3d hat(const Eigen::Vector3d & value);
Eigen::Vector3d vee(const Eigen::Matrix3d & value);
Eigen::Matrix3d expSO3(const Eigen::Vector3d & phi);
Eigen::Vector3d logSO3(const Eigen::Matrix3d & rotation);
Eigen::Matrix3d leftJacobianSO3(const Eigen::Vector3d & phi);
Eigen::Matrix3d projectSO3(const Eigen::Matrix3d & rotation);

}  // namespace lu_ommpc

#endif  // LU_OMMPC__SO3_HPP_
