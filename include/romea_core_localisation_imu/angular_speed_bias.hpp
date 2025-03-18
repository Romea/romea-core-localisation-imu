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


#ifndef ROMEA_CORE_LOCALISATION_IMU__ANGULAR_SPEED_BIAS_HPP_
#define ROMEA_CORE_LOCALISATION_IMU__ANGULAR_SPEED_BIAS_HPP_

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
namespace localisation
{

class AngularSpeedBias
{
public:
  AngularSpeedBias(
    const double & imu_rate,
    const double & acceleration_speed_std,
    const double & angular_speed_std);

  std::optional<double> evaluate(
    const double & linear_speed,
    const AccelerationsFrame & accelerations,
    const AngularSpeedsFrame & angularSpeeds);

  DiagnosticReport get_report()const;

  void reset(bool resetZeroVelocityEstimator);

private:
  bool has_null_linear_speed_(const double & linearSpeed)const;

  bool has_zero_velocity_(
    const AccelerationsFrame & accelerations,
    const AngularSpeedsFrame & angularSpeeds);

  void update_angular_speed_bias_(
    const double & linearSpeed,
    const AccelerationsFrame & accelerations,
    const AngularSpeedsFrame & angularSpeeds);

  void set_diagnostic_(const DiagnosticStatus & status, const std::string & message);

private:
  ZeroVelocityEstimator zero_velocity_;
  OnlineAverage imu_angular_speed_bias_estimator_;

  mutable std::mutex mutex_;
  DiagnosticReport report_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_IMU__ANGULAR_SPEED_BIAS_HPP_
