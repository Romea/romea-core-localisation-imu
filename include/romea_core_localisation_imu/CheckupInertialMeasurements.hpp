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

#ifndef ROMEA_CORE_LOCALISATION_IMU__CHECKUPINERTIALMEASUREMENTS_HPP_
#define ROMEA_CORE_LOCALISATION_IMU__CHECKUPINERTIALMEASUREMENTS_HPP_

// romea
#include <romea_core_imu/AccelerationsFrame.hpp>
#include <romea_core_imu/AngularSpeedsFrame.hpp>
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

// std
#include <mutex>
#include <string>

namespace romea
{
namespace core
{

class CheckupInertialMeasurements
{
public:
  CheckupInertialMeasurements(
    const double & accelerationRange,
    const double & angularSpeedRange);

  DiagnosticStatus evaluate(
    const AccelerationsFrame & accelerations,
    const AngularSpeedsFrame & angularSpeeds);

  DiagnosticReport getReport() const;

  void reset();

private:
  void checkAccelerations_(const AccelerationsFrame & accelerationFrame);
  void checkAngularSpeeds_(const AngularSpeedsFrame & angularSpeedFrame);

  void declareReportInfos_();
  void setReportInfos_(const AccelerationsFrame & accelarations);
  void setReportInfos_(const AngularSpeedsFrame & angularSpeeds);
  void addDiagnostic_(const DiagnosticStatus & status, const std::string & message);

private:
  double accelerationRange_;
  double angularSpeedRange_;

  mutable std::mutex mutex_;
  DiagnosticReport report_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_IMU__CHECKUPINERTIALMEASUREMENTS_HPP_
