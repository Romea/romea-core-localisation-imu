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


#ifndef ROMEA_CORE_LOCALISATION_IMU__LOCALISATIONIMUPLUGIN_HPP_
#define ROMEA_CORE_LOCALISATION_IMU__LOCALISATIONIMUPLUGIN_HPP_


// romea
#include <romea_core_imu/IMUAHRS.hpp>
#include <romea_core_common/log/SimpleFileLogger.hpp>
#include <romea_core_common/diagnostic/CheckupRate.hpp>
#include <romea_core_localisation/ObservationAngularSpeed.hpp>
#include <romea_core_localisation/ObservationAttitude.hpp>

// std
#include <memory>
#include <string>

// local
#include "romea_core_localisation_imu/CheckupInertialMeasurements.hpp"
#include "romea_core_localisation_imu/CheckupAttitude.hpp"
#include "romea_core_localisation_imu/AngularSpeedBias.hpp"

namespace romea
{
namespace core
{

class LocalisationIMUPlugin
{
public:
  explicit LocalisationIMUPlugin(std::unique_ptr<IMUAHRS> imu);

  void enableDebugLog(const std::string & logFilename);

  void processLinearSpeed(
    const Duration & stamp,
    const double & linearSpeed);

  bool computeAngularSpeed(
    const Duration & stamp,
    const double & accelerationAlongXAxis,
    const double & accelerationAlongYAxis,
    const double & accelerationAlongZAxis,
    const double & angularSpeedAroundXAxis,
    const double & angularSpeedAroundYAxis,
    const double & angularSpeedAroundZAxis,
    ObservationAngularSpeed & angularSpeed);

  bool computeAttitude(
    const Duration & stamp,
    const double & rollAngle,
    const double & pitchAngle,
    const double & courseAngle,
    ObservationAttitude & attitude);

  DiagnosticReport makeDiagnosticReport(const Duration & stamp);

private:
  void checkHeartBeats_(const Duration & stamp);

  DiagnosticReport makeDiagnosticReport_();

private:
  std::unique_ptr<IMUAHRS> imu_;
  AngularSpeedBias imuAngularSpeedBias_;
  std::atomic<double> linearSpeed_;

  CheckupGreaterThanRate attitudeRateDiagnostic_;
  CheckupGreaterThanRate linearSpeedRateDiagnostic_;
  CheckupGreaterThanRate inertialMeasurementRateDiagnostic_;

  CheckupAttitude attitudeDiagnostic_;
  CheckupInertialMeasurements inertialMeasurementDiagnostic_;

  SimpleFileLogger debugLogger_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_IMU__LOCALISATIONIMUPLUGIN_HPP_
