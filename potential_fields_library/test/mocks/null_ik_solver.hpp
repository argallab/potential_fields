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

#ifndef NULL_IK_SOLVER_HPP
#define NULL_IK_SOLVER_HPP

#include "solvers/ik_solver.hpp"
#include <vector>
#include <string>
#include <Eigen/Dense>

// A minimal IK solver that returns the seed as the solution (useful for offline testing)
class NullIKSolver : public pfield::IKSolver {
public:
  NullIKSolver() : pfield::IKSolver("NullIKSolver") {}
  ~NullIKSolver() override = default;

  bool solve([[maybe_unused]] const Eigen::Isometry3d& targetPose, const std::vector<double>& seed,
    std::vector<double>& solution, Eigen::Matrix<double, 6, Eigen::Dynamic>& J, std::string& errorMsg) override {
    if (seed.empty()) { solution = {}; }
    else { solution = seed; }
    J.resize(6, solution.size());
    J.setZero();
    errorMsg.clear();
    return true;
  }

  bool computeJacobian(const std::vector<double>& jointPositions, Eigen::Matrix<double, 6, Eigen::Dynamic>& J) override {
    J.resize(6, jointPositions.size());
    J.setZero();
    return true;
  }

  std::vector<std::string> getJointNames() const override { return {}; }

  std::vector<double> getHomeConfiguration() const override { return {}; }
};

#endif // NULL_IK_SOLVER_HPP
