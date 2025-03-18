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
#include <limits>
#include <memory>
#include <utility>
#include <string>

// local
#include "romea_core_localisation_imu/imu_plugin.hpp"

namespace
{
const double LINEAR_SPEED_EPSILON = 0.001;
}

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
IMUPlugin::IMUPlugin(std::unique_ptr<IMUAHRS> imu)
: imu_(std::move(imu)),
  imu_angular_speed_bias_(imu_->getRate(),
    imu_->getAccelerationStd(),
    imu_->getAngularSpeedStd()),
  linear_speed_(std::numeric_limits<double>::quiet_NaN()),
  attitude_rate_diagnostic_("attitude",
    imu_->getRate(),
    imu_->getRate() * 0.1),
  linear_speed_rate_diagnostic_("linear_speed", 10.0, 1.),
  inertial_measurement_rate_diagnostic_("inertial_measurements",
    imu_->getRate(),
    imu_->getRate() * 0.1),
  attitude_diagnostic_(),
  inertial_measurement_diagnostic_(imu_->getAccelerationRange(),
    imu_->getAngularSpeedRange()),
  debug_logger_()
{
}

//-----------------------------------------------------------------------------
void IMUPlugin::enable_debug_log(const std::string & logFilename)
{
  debug_logger_.init(logFilename);
}

//-----------------------------------------------------------------------------
void IMUPlugin::process_linear_speed(
  const Duration & stamp,
  const double & linearSpeed)
{
  if (linear_speed_rate_diagnostic_.evaluate(stamp) == DiagnosticStatus::OK) {
    linear_speed_.store(linearSpeed);
  }
}

//-----------------------------------------------------------------------------
bool IMUPlugin::compute_angular_speed(
  const Duration & stamp,
  const double & accelerationAlongXAxis,
  const double & accelerationAlongYAxis,
  const double & accelerationAlongZAxis,
  const double & angularSpeedAroundXAxis,
  const double & angularSpeedAroundYAxis,
  const double & angularSpeedAroundZAxis,
  ObservationAngularSpeed & angularSpeed)
{
  AccelerationsFrame accelerations =
    imu_->createAccelerationsFrame(
    accelerationAlongXAxis,
    accelerationAlongYAxis,
    accelerationAlongZAxis);


  AngularSpeedsFrame angularSpeeds =
    imu_->createAngularSpeedsFrame(
    angularSpeedAroundXAxis,
    angularSpeedAroundYAxis,
    angularSpeedAroundZAxis);

  if (inertial_measurement_rate_diagnostic_.evaluate(stamp) == DiagnosticStatus::OK &&
    inertial_measurement_diagnostic_.evaluate(accelerations, angularSpeeds) == DiagnosticStatus::OK)
  {
    auto angularSpeedBias = imu_angular_speed_bias_.
      evaluate(linear_speed_.load(), accelerations, angularSpeeds);

    if (angularSpeedBias.has_value()) {
      angularSpeed.Y() = angularSpeeds.angularSpeedAroundZAxis - angularSpeedBias.value();
      angularSpeed.R() = imu_->getAngularSpeedVariance();
      return true;
    }
  }
  return false;
}


//-----------------------------------------------------------------------------
bool IMUPlugin::compute_attitude(
  const Duration & stamp,
  const double & rollAngle,
  const double & pitchAngle,
  const double & courseAngle,
  ObservationAttitude & attitude)
{
  RollPitchCourseFrame frame = imu_->createFrame(
    rollAngle,
    pitchAngle,
    courseAngle);

  if (attitude_rate_diagnostic_.evaluate(stamp) == DiagnosticStatus::OK &&
    attitude_diagnostic_.evaluate(frame) == DiagnosticStatus::OK)
  {
    attitude.Y(ObservationAttitude::ROLL) = rollAngle;
    attitude.Y(ObservationAttitude::PITCH) = pitchAngle;
    attitude.R() = Eigen::Matrix2d::Identity() * imu_->getAngleVariance();
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
DiagnosticReport IMUPlugin::make_diagnostic_report(const Duration & stamp)
{
  check_heart_beats_(stamp);
  return make_diagnostic_report_();
}

//-----------------------------------------------------------------------------
void IMUPlugin::check_heart_beats_(const Duration & stamp)
{
  if (!attitude_rate_diagnostic_.heartBeatCallback(stamp)) {
    attitude_diagnostic_.reset();
  }

  if (!linear_speed_rate_diagnostic_.heartBeatCallback(stamp)) {
    linear_speed_ = std::numeric_limits<double>::quiet_NaN();
    imu_angular_speed_bias_.reset(false);
  }

  if (!inertial_measurement_rate_diagnostic_.heartBeatCallback(stamp)) {
    inertial_measurement_diagnostic_.reset();
    imu_angular_speed_bias_.reset(true);
  }
}

//-----------------------------------------------------------------------------
DiagnosticReport IMUPlugin::make_diagnostic_report_()
{
  DiagnosticReport report;
  report += linear_speed_rate_diagnostic_.getReport();
  report += attitude_rate_diagnostic_.getReport();
  report += attitude_diagnostic_.get_report();
  report += inertial_measurement_rate_diagnostic_.getReport();
  report += inertial_measurement_diagnostic_.get_report();
  report += imu_angular_speed_bias_.get_report();
  return report;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
