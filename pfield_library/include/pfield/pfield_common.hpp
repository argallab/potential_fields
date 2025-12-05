#ifndef PFIELD_COMMON_HPP_
#define PFIELD_COMMON_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
// URDF types used by common helpers (e.g., pose conversion)
#include <urdf_model/model.h>

namespace pfield {

  inline static bool isPositiveFinite(double v) { return std::isfinite(v) && v > 1e-12; }

  /**
   * @brief Given a vector (typically linear or angular velocity), a maximum norm,
   *        and an optional 'softness' parameter beta, applies a soft saturation
   *
   * @param v The 3D vector to be saturated
   * @param maxNorm The maximum allowed norm
   * @param beta Softness parameter for the saturation, higher = more abrupt (default=1.0)
   * @return Eigen::Vector3d The saturated vector
   */
  inline static Eigen::Vector3d softSaturateNorm(const Eigen::Vector3d& v, double maxNorm, double beta = 1.0) {
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
  inline Eigen::Vector3d rateLimitStep(const Eigen::Vector3d& prev, const Eigen::Vector3d& curr, double dMax) {
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
  static inline Eigen::Vector3d rotateVector(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) { return q * v; }

  /**
   * @brief Convert a urdf::Pose to an Eigen::Affine3d
   *
   * The URDF pose stores position (x,y,z) and a quaternion (x,y,z,w). This helper
   * creates an Eigen isometry with the same translation and rotation.
   */
  static inline Eigen::Affine3d urdfPoseToEigen(const urdf::Pose& p) {
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
  static inline Eigen::VectorXd dampedLeastSquares(
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
