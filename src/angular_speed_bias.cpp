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


// std
#include <string>

// local
#include "romea_core_localisation_imu/angular_speed_bias.hpp"

namespace
{
const double ANGULAR_SPEED_BIAS_EPSILON = 0.0001;
const double LINEAR_SPEED_EPSILON = 0.02;
}

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
AngularSpeedBias::AngularSpeedBias(
  const double & imuRate,
  const double & accelerationSpeedStd,
  const double & angularSpeedStd)
: zero_velocity_(imuRate, accelerationSpeedStd, angularSpeedStd),
  imu_angular_speed_bias_estimator_(ANGULAR_SPEED_BIAS_EPSILON, 5 * imuRate),
  mutex_(),
  report_()
{
  setReportInfo(report_, "acceleration_std", "");
  setReportInfo(report_, "angular_speed_std", "");
  setReportInfo(report_, "linear_speed", "");
  setReportInfo(report_, "angular_speed_bias", "");
}

//-----------------------------------------------------------------------------
bool AngularSpeedBias::has_null_linear_speed_(const double & linear_speed)const
{
  return std::isfinite(linear_speed) && std::abs(linear_speed) < LINEAR_SPEED_EPSILON;
}

//-----------------------------------------------------------------------------
bool AngularSpeedBias::has_zero_velocity_(
  const AccelerationsFrame & accelerations,
  const AngularSpeedsFrame & angularSpeeds)
{
  return zero_velocity_.update(
    accelerations.accelerationAlongXAxis,
    accelerations.accelerationAlongYAxis,
    accelerations.accelerationAlongZAxis,
    angularSpeeds.angularSpeedAroundXAxis,
    angularSpeeds.angularSpeedAroundYAxis,
    angularSpeeds.angularSpeedAroundZAxis);
}


//-----------------------------------------------------------------------------
void AngularSpeedBias::update_angular_speed_bias_(
  const double & linearSpeed,
  const AccelerationsFrame & accelerations,
  const AngularSpeedsFrame & angularSpeeds)
{
  bool hasNullLinearSpeed = has_null_linear_speed_(linearSpeed);
  bool hasZeroVelocity = has_zero_velocity_(accelerations, angularSpeeds);

  if (hasZeroVelocity && hasNullLinearSpeed) {
    imu_angular_speed_bias_estimator_.update(angularSpeeds.angularSpeedAroundZAxis);
  }
}


//-----------------------------------------------------------------------------
std::optional<double> AngularSpeedBias::evaluate(
  const double & linearSpeed,
  const AccelerationsFrame & accelerations,
  const AngularSpeedsFrame & angularSpeeds)
{
  update_angular_speed_bias_(linearSpeed, accelerations, angularSpeeds);

  std::lock_guard<std::mutex> lock(mutex_);
  setReportInfo(report_, "acceleration_std", zero_velocity_.getAccelerationStd());
  setReportInfo(report_, "angular_speed_std", zero_velocity_.getAngularSpeedStd());
  setReportInfo(
    report_, "linear_speed", std::isfinite(linearSpeed) ? std::to_string(
      linearSpeed) : "");

  if (imu_angular_speed_bias_estimator_.isAvailable()) {
    double angularSpeedBias = imu_angular_speed_bias_estimator_.getAverage();
    set_diagnostic_(DiagnosticStatus::OK, "Angular speed bias is OK.");
    setReportInfo(report_, "angular_speed_bias", angularSpeedBias);
    return angularSpeedBias;
  } else {
    set_diagnostic_(DiagnosticStatus::WARN, "Angular speed bias not available.");
    setReportInfo(report_, "angular_speed_bias", "");
    return std::nullopt;
  }
}

//-----------------------------------------------------------------------------
void AngularSpeedBias::reset(bool resetZeroVelocityEstimator)
{
  std::lock_guard<std::mutex> lock(mutex_);
  report_.diagnostics.clear();

  if (resetZeroVelocityEstimator) {
    zero_velocity_.reset();
    setReportInfo(report_, "acceleration_std", "");
    setReportInfo(report_, "angular_speed_std", "");
  } else {
    setReportInfo(report_, "linear_speed", "");
  }

  imu_angular_speed_bias_estimator_.reset();
  setReportInfo(report_, "angular_speed_bias", "");
  setDiagnostic_(DiagnosticStatus::WARN, "Angular speed bias not available.");
}

//-----------------------------------------------------------------------------
void AngularSpeedBias::set_diagnostic_(
  const DiagnosticStatus & status,
  const std::string & message)
{
  report_.diagnostics.clear();
  report_.diagnostics.push_back({status, message});
}

//-----------------------------------------------------------------------------
DiagnosticReport AngularSpeedBias::get_report()const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return report_;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
