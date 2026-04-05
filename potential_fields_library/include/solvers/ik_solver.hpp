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

#ifndef IK_SOLVER_HPP
#define IK_SOLVER_HPP

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pfield {

  // IKSolver Interface for different IK solver implementations
  class IKSolver {
  public:
    explicit IKSolver(const std::string& name) : name(name) {}
    virtual ~IKSolver() = default;

    // Primary IK (targetPose is base->EE as an Isometry) and Jacobian function
    virtual bool solve(
      const Eigen::Isometry3d& targetPose,
      const std::vector<double>& seed,
      std::vector<double>& solution,
      Eigen::Matrix<double, 6, Eigen::Dynamic>& J,
      std::string& errorMsg) = 0;

    // Optional Jacobian-only function (for joint velocities from Cartesian twist)
    virtual bool computeJacobian(
      const std::vector<double>& jointPositions,
      Eigen::Matrix<double, 6, Eigen::Dynamic>& J) = 0;

    [[nodiscard]] virtual std::vector<double> getHomeConfiguration() const = 0;

    [[nodiscard]] virtual std::vector<std::string> getJointNames() const = 0;

    [[nodiscard]] virtual std::string getName() const { return this->name; }

  protected:
    std::string name;
  };

} // namespace pfield

#endif // IK_SOLVER_HPP
