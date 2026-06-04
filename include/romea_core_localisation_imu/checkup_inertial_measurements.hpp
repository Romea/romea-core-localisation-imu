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

#ifndef ROMEA_CORE_LOCALISATION_IMU__CHECKUP_INERTIAL_MEASUREMENTS_HPP_
#define ROMEA_CORE_LOCALISATION_IMU__CHECKUP_INERTIAL_MEASUREMENTS_HPP_

// romea
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>
#include <romea_core_imu/AccelerationsFrame.hpp>
#include <romea_core_imu/AngularSpeedsFrame.hpp>

// std
#include <mutex>
#include <string>

namespace romea
{
namespace core
{
namespace localisation
{

class CheckupInertialMeasurements
{
public:
  CheckupInertialMeasurements(
    const double & acceleration_range, const double & angular_speed_range);

  DiagnosticStatus evaluate(
    const AccelerationsFrame & accelerations, const AngularSpeedsFrame & angular_speeds);

  DiagnosticReport get_report() const;

  void reset();

private:
  void check_accelerations_(const AccelerationsFrame & acceleration_frame);
  void check_angular_speeds_(const AngularSpeedsFrame & angular_speed_frame);

  void declare_report_infos_();
  void set_report_infos_(const AccelerationsFrame & accelarations);
  void set_report_infos_(const AngularSpeedsFrame & angular_speeds);
  void add_diagnostic_(const DiagnosticStatus & status, const std::string & message);

private:
  double acceleration_range_;
  double angular_speed_range_;

  mutable std::mutex mutex_;
  DiagnosticReport report_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_IMU__CHECKUP_INERTIAL_MEASUREMENTS_HPP_
