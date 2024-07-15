// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


#ifndef ROMEA_CORE_LOCALISATION_IMU__ANGULARSPEEDBIAS_HPP_
#define ROMEA_CORE_LOCALISATION_IMU__ANGULARSPEEDBIAS_HPP_

// romea
#include <romea_core_imu/algorithms/ZeroVelocityEstimator.hpp>
#include <romea_core_imu/AccelerationsFrame.hpp>
#include <romea_core_imu/AngularSpeedsFrame.hpp>
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

// std
#include <optional>
#include <string>
#include <mutex>


namespace romea
{
namespace core
{

class AngularSpeedBias
{
public:
  AngularSpeedBias(
    const double & imuRate,
    const double & accelerationSpeedStd,
    const double & angularSpeedStd);

  std::optional<double> evaluate(
    const double & linearSpeed,
    const AccelerationsFrame & accelerations,
    const AngularSpeedsFrame & angularSpeeds);

  DiagnosticReport getReport()const;

  void reset(bool resetZeroVelocityEstimator);

private:
  bool hasNullLinearSpeed_(const double & linearSpeed)const;

  bool hasZeroVelocity_(
    const AccelerationsFrame & accelerations,
    const AngularSpeedsFrame & angularSpeeds);

  void updateAngularSpeedBias_(
    const double & linearSpeed,
    const AccelerationsFrame & accelerations,
    const AngularSpeedsFrame & angularSpeeds);

  void setDiagnostic_(const DiagnosticStatus & status, const std::string & message);

private:
  ZeroVelocityEstimator zeroVelocity_;
  OnlineAverage imuAngularSpeedBiasEstimator_;

  mutable std::mutex mutex_;
  DiagnosticReport report_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_IMU__ANGULARSPEEDBIAS_HPP_
