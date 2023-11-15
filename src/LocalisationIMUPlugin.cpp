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
#include "romea_core_localisation_imu/LocalisationIMUPlugin.hpp"

namespace
{
const double LINEAR_SPEED_EPSILON = 0.001;
}


namespace romea
{

//-----------------------------------------------------------------------------
LocalisationIMUPlugin::LocalisationIMUPlugin(std::unique_ptr<IMUAHRS> imu)
: imu_(std::move(imu)),
  imuAngularSpeedBias_(imu_->getRate(),
    imu_->getAccelerationStd(),
    imu_->getAngularSpeedStd()),
  linearSpeed_(std::numeric_limits<double>::quiet_NaN()),
  attitudeRateDiagnostic_("attitude",
    imu_->getRate(),
    imu_->getRate() * 0.1),
  linearSpeedRateDiagnostic_("linear_speed", 10.0, 1.),
  inertialMeasurementRateDiagnostic_("inertial_measurements",
    imu_->getRate(),
    imu_->getRate() * 0.1),
  attitudeDiagnostic_(),
  inertialMeasurementDiagnostic_(imu_->getAccelerationRange(),
    imu_->getAngularSpeedRange()),
  debugLogger_()
{
}

//-----------------------------------------------------------------------------
void LocalisationIMUPlugin::enableDebugLog(const std::string & logFilename)
{
  debugLogger_.init(logFilename);
}

//-----------------------------------------------------------------------------
void LocalisationIMUPlugin::processLinearSpeed(
  const Duration & stamp,
  const double & linearSpeed)
{
  if (linearSpeedRateDiagnostic_.evaluate(stamp) == DiagnosticStatus::OK) {
    linearSpeed_.store(linearSpeed);
  }
}

//-----------------------------------------------------------------------------
bool LocalisationIMUPlugin::computeAngularSpeed(
  const Duration & stamp,
  const double & accelerationAlongXAxis,
  const double & accelerationAlongYAxis,
  const double & accelerationAlongZAxis,
  const double & angularSpeedAroundXAxis,
  const double & angularSpeedAroundYAxis,
  const double & angularSpeedAroundZAxis,
  ObservationAngularSpeed & angularSpeed)
{
  romea::AccelerationsFrame accelerations =
    imu_->createAccelerationsFrame(
    accelerationAlongXAxis,
    accelerationAlongYAxis,
    accelerationAlongZAxis);


  romea::AngularSpeedsFrame angularSpeeds =
    imu_->createAngularSpeedsFrame(
    angularSpeedAroundXAxis,
    angularSpeedAroundYAxis,
    angularSpeedAroundZAxis);

  if (inertialMeasurementRateDiagnostic_.evaluate(stamp) == DiagnosticStatus::OK &&
    inertialMeasurementDiagnostic_.evaluate(accelerations, angularSpeeds) == DiagnosticStatus::OK)
  {
    auto angularSpeedBias = imuAngularSpeedBias_.
      evaluate(linearSpeed_.load(), accelerations, angularSpeeds);

    if (angularSpeedBias.has_value()) {
      angularSpeed.Y() = angularSpeeds.angularSpeedAroundZAxis - angularSpeedBias.value();
      angularSpeed.R() = imu_->getAngularSpeedVariance();
      return true;
    }
  }
  return false;
}


//-----------------------------------------------------------------------------
bool LocalisationIMUPlugin::computeAttitude(
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

  if (attitudeRateDiagnostic_.evaluate(stamp) == DiagnosticStatus::OK &&
    attitudeDiagnostic_.evaluate(frame) == DiagnosticStatus::OK)
  {
    attitude.Y(ObservationAttitude::ROLL) = rollAngle;
    attitude.Y(ObservationAttitude::PITCH) = pitchAngle;
    attitude.R() = Eigen::Matrix2d::Identity() * imu_->getAngleVariance();
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
DiagnosticReport LocalisationIMUPlugin::makeDiagnosticReport(const Duration & stamp)
{
  checkHeartBeats_(stamp);
  return makeDiagnosticReport_();
}

//-----------------------------------------------------------------------------
void LocalisationIMUPlugin::checkHeartBeats_(const Duration & stamp)
{
  if (!attitudeRateDiagnostic_.heartBeatCallback(stamp)) {
    attitudeDiagnostic_.reset();
  }

  if (!linearSpeedRateDiagnostic_.heartBeatCallback(stamp)) {
    linearSpeed_ = std::numeric_limits<double>::quiet_NaN();
    imuAngularSpeedBias_.reset(false);
  }

  if (!inertialMeasurementRateDiagnostic_.heartBeatCallback(stamp)) {
    inertialMeasurementDiagnostic_.reset();
    imuAngularSpeedBias_.reset(true);
  }
}

//-----------------------------------------------------------------------------
DiagnosticReport LocalisationIMUPlugin::makeDiagnosticReport_()
{
  DiagnosticReport report;
  report += linearSpeedRateDiagnostic_.getReport();
  report += attitudeRateDiagnostic_.getReport();
  report += attitudeDiagnostic_.getReport();
  report += inertialMeasurementRateDiagnostic_.getReport();
  report += inertialMeasurementDiagnostic_.getReport();
  report += imuAngularSpeedBias_.getReport();
  return report;
}

}  // namespace romea
