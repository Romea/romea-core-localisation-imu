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
#include "romea_core_localisation_imu/CheckupInertialMeasurements.hpp"

namespace romea
{
namespace core
{

//-----------------------------------------------------------------------------
CheckupInertialMeasurements::CheckupInertialMeasurements(
  const double & accelerationRange,
  const double & angularSpeedRange)
: accelerationRange_(accelerationRange),
  angularSpeedRange_(angularSpeedRange),
  report_()
{
  declareReportInfos_();
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::declareReportInfos_()
{
  setReportInfo(report_, "acceleration_x", "");
  setReportInfo(report_, "acceleration_y", "");
  setReportInfo(report_, "acceleration_z", "");
  setReportInfo(report_, "angular_speed_x", "");
  setReportInfo(report_, "angular_speed_y", "");
  setReportInfo(report_, "angular_speed_z", "");
}

//-----------------------------------------------------------------------------
DiagnosticStatus CheckupInertialMeasurements::evaluate(
  const AccelerationsFrame & accelerations,
  const AngularSpeedsFrame & angularSpeeds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  report_.diagnostics.clear();
  checkAccelerations_(accelerations);
  checkAngularSpeeds_(angularSpeeds);
  setReportInfos_(accelerations);
  setReportInfos_(angularSpeeds);
  return worseStatus(report_.diagnostics);
}

//--------------------------------------------------------------------
void CheckupInertialMeasurements::checkAccelerations_(const AccelerationsFrame & accelerationFrame)
{
  if (std::abs(accelerationFrame.accelerationAlongXAxis) > accelerationRange_ ||
    std::abs(accelerationFrame.accelerationAlongYAxis) > accelerationRange_ ||
    std::abs(accelerationFrame.accelerationAlongZAxis) > accelerationRange_)
  {
    addDiagnostic_(DiagnosticStatus::ERROR, "Acceleration data is out of range.");
  } else {
    addDiagnostic_(DiagnosticStatus::OK, "Acceleration data is OK.");
  }
}

//--------------------------------------------------------------------
void CheckupInertialMeasurements::checkAngularSpeeds_(const AngularSpeedsFrame & angularSpeedFrame)
{
  if (std::abs(angularSpeedFrame.angularSpeedAroundXAxis) > angularSpeedRange_ ||
    std::abs(angularSpeedFrame.angularSpeedAroundYAxis) > angularSpeedRange_ ||
    std::abs(angularSpeedFrame.angularSpeedAroundZAxis) > angularSpeedRange_)
  {
    addDiagnostic_(DiagnosticStatus::ERROR, "Angular speed data is out of range.");
  } else {
    addDiagnostic_(DiagnosticStatus::OK, "Angular speed data is OK.");
  }
}


//-----------------------------------------------------------------------------
DiagnosticReport CheckupInertialMeasurements::getReport() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return report_;
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::setReportInfos_(const AccelerationsFrame & accelarations)
{
  setReportInfo(report_, "acceleration_x", accelarations.accelerationAlongXAxis);
  setReportInfo(report_, "acceleration_y", accelarations.accelerationAlongYAxis);
  setReportInfo(report_, "acceleration_z", accelarations.accelerationAlongZAxis);
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::setReportInfos_(const AngularSpeedsFrame & angularSpeeds)
{
  setReportInfo(report_, "angular_speed_x", angularSpeeds.angularSpeedAroundXAxis);
  setReportInfo(report_, "angular_speed_y", angularSpeeds.angularSpeedAroundYAxis);
  setReportInfo(report_, "angular_speed_z", angularSpeeds.angularSpeedAroundZAxis);
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::addDiagnostic_(
  const DiagnosticStatus & status,
  const std::string & message)
{
  Diagnostic diagnostic;
  diagnostic.message = message;
  diagnostic.status = status;
  report_.diagnostics.push_back(diagnostic);
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::reset()
{
  report_.diagnostics.clear();
  declareReportInfos_();
}

}  // namespace core
}  // namespace romea
