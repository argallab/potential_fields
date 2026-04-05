// Copyright 2025 Sharwin Patil
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

#ifndef PFIELD_COMMON_HPP_
#define PFIELD_COMMON_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
// URDF types used by common helpers (e.g., pose conversion)
#include <urdf_model/model.h>
#include <vector>

namespace pfield {

  [[nodiscard]] inline bool isPositiveFinite(double v) { return std::isfinite(v) && v > 1e-12; }

  /**
   * @brief Given a vector (typically linear or angular velocity), a maximum norm,
   *        and an optional 'softness' parameter beta, applies a soft saturation
   *
   * @param v The 3D vector to be saturated
   * @param maxNorm The maximum allowed norm
   * @param beta Softness parameter for the saturation, higher = more abrupt (default=1.0)
   * @return Eigen::Vector3d The saturated vector
   */
  [[nodiscard]] inline Eigen::Vector3d softSaturateNorm(const Eigen::Vector3d& v, double maxNorm, double beta = 1.0) {
    if (!isPositiveFinite(maxNorm)) return v;
    const double n = v.norm();
    if (!isPositiveFinite(n)) return v;
    // scale is between [0, 1], beta controls how "soft" the smoothing is
    // higher beta results in steeper but still smooth curve near maxNorm
    const double s = (maxNorm * std::tanh(beta * n / maxNorm)) / n;
    return v * s;
  }


  /**
   * @brief Limits the step from a previous vector (typically velocity) to another vector
   *
   * @param prev The previous vector
   * @param curr The current vector to compute the step towards
   * @param dMax The   maximum allowed step size
   * @return Eigen::Vector3d The rate-limited vector
   */
  [[nodiscard]] inline Eigen::Vector3d rateLimitStep(const Eigen::Vector3d& prev, const Eigen::Vector3d& curr, double dMax) {
    if (!isPositiveFinite(dMax) || dMax <= 0.0) return curr;
    Eigen::Vector3d d = curr - prev;
    const double dn = d.norm();
    if (!isPositiveFinite(dn) || dn <= dMax) return curr;
    return prev + (d * (dMax / dn));
  }

  /**
   * @brief Rotates the given vector v by the quaternion q
   *
   * @param q The quaternion representing the rotation
   * @param v The 3D Vector to be rotated
   * @return Eigen::Vector3d The rotated vector
   */
  [[nodiscard]] inline Eigen::Vector3d rotateVector(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) { return q * v; }

  /**
   * @brief Convert a urdf::Pose to an Eigen::Affine3d
   *
   * The URDF pose stores position (x,y,z) and a quaternion (x,y,z,w). This helper
   * creates an Eigen isometry with the same translation and rotation.
   */
  [[nodiscard]] inline Eigen::Affine3d urdfPoseToEigen(const urdf::Pose& p) {
    const Eigen::Quaterniond q(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
    const Eigen::Translation3d t(p.position.x, p.position.y, p.position.z);
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.linear() = q.toRotationMatrix();
    T.translation() = t.vector();
    return T;
  }

  /**
   * @brief Given a Jacobian Matrix J and a 6D twist V, compute the damped least squares solution
   *        for the joint velocities.
   *
   * @param J The Jacobian matrix, size (6 x n)
   * @param V The 6D twist vector
   * @param lambda The damping factor for the least squares solution
   * @return Eigen::VectorXd The computed joint velocities
   */
  [[nodiscard]] inline Eigen::VectorXd dampedLeastSquares(
    const Eigen::MatrixXd& J,
    const Eigen::VectorXd& V,
    const double lambda = 1e-3) {
    // q_dot = J.T * (J * J.T + lambda^2 * I)^-1 * V
    const int m = static_cast<int>(J.rows());
    Eigen::MatrixXd JJt = J * J.transpose();
    JJt += lambda * lambda * Eigen::MatrixXd::Identity(m, m);

    Eigen::VectorXd y = JJt.ldlt().solve(V);
    return J.transpose() * y;
  }

  /**
   * @brief Approximates joint velocities given current and previous joint angles and a time step
   *
   * @note This function uses finite differencing to estimate joint velocities which may be noisy and
   *       inaccurate for small dt values. This can likely be improved with filtering or by providing more
   *       historical velocity data to compute a better estimate.
   *
   * @param currentJointAngles The current joint angles [rad]
   * @param previousJointAngles The previous joint angles [rad]
   * @param dt The time difference between the current and previous joint angles [s]
   * @return std::vector<double> The approximated joint velocities [rad/s]
   */
  inline std::vector<double> approximateJointVelocities(
    const std::vector<double>& currentJointAngles,
    const std::vector<double>& previousJointAngles,
    const double dt) {
    std::vector<double> jointVelocities;
    if (currentJointAngles.size() != previousJointAngles.size() || dt <= 0.0) {
      return jointVelocities; // return empty on error
    }
    jointVelocities.reserve(currentJointAngles.size());
    for (size_t i = 0; i < currentJointAngles.size(); ++i) {
      jointVelocities.push_back((currentJointAngles[i] - previousJointAngles[i]) / dt);
    }
    return jointVelocities;
  }

  constexpr double DEFAULT_ATTRACTIVE_GAIN = 1.0; // Gain for attractive force [Ns/m]
  constexpr double DEFAULT_ROTATIONAL_ATTRACTIVE_GAIN = 0.7; // Gain for rotational attractive force [Ns/m]
  constexpr double DEFAULT_MAX_LINEAR_VELOCITY = 5.0; // [m/s]
  constexpr double DEFAULT_MAX_ANGULAR_VELOCITY = 1.0; // [rad/s]
  constexpr double DEFAULT_MAX_LINEAR_ACCELERATION = 1.0; // [m/s^2]
  constexpr double DEFAULT_MAX_ANGULAR_ACCELERATION = 1.0; // [rad/s^2]
  constexpr double DEFAULT_REPULSIVE_GAIN = 0.22; // [Ns/m] default repulsive gain
  constexpr double DEFAULT_INFLUENCE_DISTANCE = 1.0; // [m] default influence distance
  constexpr double NEAR_ZERO_THRESHOLD = 1e-9; // Threshold for floating point near-zero comparisons

} // namespace pfield

#endif // !PFIELD_COMMON_HPP_
