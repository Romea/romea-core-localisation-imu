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
#include "romea_core_localisation_imu/checkup_attitude.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

//-----------------------------------------------------------------------------
CheckupAttitude::CheckupAttitude() : report_()
{
}

//-----------------------------------------------------------------------------
void CheckupAttitude::declare_report_infos_()
{
  setReportInfo(report_, "roll", "");
  setReportInfo(report_, "pitch", "");
}

//-----------------------------------------------------------------------------
DiagnosticStatus CheckupAttitude::evaluate(const RollPitchCourseFrame & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (check_attitude_angles_(frame)) {
    set_diagnostic_(DiagnosticStatus::OK, "Attitude is OK.");
  } else {
    set_diagnostic_(DiagnosticStatus::ERROR, "Attitude angles are out of range.");
  }

  set_report_infos_(frame);
  return report_.diagnostics.front().status;
}

//-----------------------------------------------------------------------------
void CheckupAttitude::set_diagnostic_(const DiagnosticStatus & status, const std::string & message)
{
  report_.diagnostics.clear();
  report_.diagnostics.push_back({status, message});
}

//-----------------------------------------------------------------------------
bool CheckupAttitude::check_attitude_angles_(const RollPitchCourseFrame & frame)
{
  return frame.rollAngle >= -M_PI_2 && frame.rollAngle <= M_PI_2 && frame.pitchAngle >= -M_PI_2 &&
         frame.pitchAngle <= M_PI_2;
}

//-----------------------------------------------------------------------------
void CheckupAttitude::set_report_infos_(const RollPitchCourseFrame & frame)
{
  setReportInfo(report_, "roll", frame.rollAngle);
  setReportInfo(report_, "pitch", frame.pitchAngle);
}

//-----------------------------------------------------------------------------
void CheckupAttitude::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  report_.diagnostics.clear();
  declare_report_infos_();
}

//-----------------------------------------------------------------------------
DiagnosticReport CheckupAttitude::get_report() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return report_;
}

}  // namespace localisation
}  // namespace core
}  // namespace romea
