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
#include "romea_core_localisation_imu/checkup_inertial_measurements.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
CheckupInertialMeasurements::CheckupInertialMeasurements(
  const double & accelerationRange, const double & angularSpeedRange)
: acceleration_range_(accelerationRange), angular_speed_range_(angularSpeedRange), report_()
{
  declare_report_infos_();
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::declare_report_infos_()
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
  const AccelerationsFrame & accelerations, const AngularSpeedsFrame & angular_speeds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  report_.diagnostics.clear();
  check_accelerations_(accelerations);
  check_angular_speeds_(angular_speeds);
  set_report_infos_(accelerations);
  set_report_infos_(angular_speeds);
  return worseStatus(report_.diagnostics);
}

//--------------------------------------------------------------------
void CheckupInertialMeasurements::check_accelerations_(const AccelerationsFrame & accelerationFrame)
{
  if (
    std::abs(accelerationFrame.accelerationAlongXAxis) > acceleration_range_ ||
    std::abs(accelerationFrame.accelerationAlongYAxis) > acceleration_range_ ||
    std::abs(accelerationFrame.accelerationAlongZAxis) > acceleration_range_) {
    add_diagnostic_(DiagnosticStatus::ERROR, "Acceleration data is out of range.");
  } else {
    add_diagnostic_(DiagnosticStatus::OK, "Acceleration data is OK.");
  }
}

//--------------------------------------------------------------------
void CheckupInertialMeasurements::check_angular_speeds_(
  const AngularSpeedsFrame & angularSpeedFrame)
{
  if (
    std::abs(angularSpeedFrame.angularSpeedAroundXAxis) > angular_speed_range_ ||
    std::abs(angularSpeedFrame.angularSpeedAroundYAxis) > angular_speed_range_ ||
    std::abs(angularSpeedFrame.angularSpeedAroundZAxis) > angular_speed_range_) {
    add_diagnostic_(DiagnosticStatus::ERROR, "Angular speed data is out of range.");
  } else {
    add_diagnostic_(DiagnosticStatus::OK, "Angular speed data is OK.");
  }
}

//-----------------------------------------------------------------------------
DiagnosticReport CheckupInertialMeasurements::get_report() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return report_;
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::set_report_infos_(const AccelerationsFrame & accelarations)
{
  setReportInfo(report_, "acceleration_x", accelarations.accelerationAlongXAxis);
  setReportInfo(report_, "acceleration_y", accelarations.accelerationAlongYAxis);
  setReportInfo(report_, "acceleration_z", accelarations.accelerationAlongZAxis);
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::set_report_infos_(const AngularSpeedsFrame & angularSpeeds)
{
  setReportInfo(report_, "angular_speed_x", angularSpeeds.angularSpeedAroundXAxis);
  setReportInfo(report_, "angular_speed_y", angularSpeeds.angularSpeedAroundYAxis);
  setReportInfo(report_, "angular_speed_z", angularSpeeds.angularSpeedAroundZAxis);
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::add_diagnostic_(
  const DiagnosticStatus & status, const std::string & message)
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
  declare_report_infos_();
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
