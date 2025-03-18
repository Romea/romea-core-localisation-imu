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
#include <romea_core_localisation/observation_angular_speed.hpp>
#include <romea_core_localisation/observation_attitude.hpp>

// std
#include <memory>
#include <string>

// local
#include "romea_core_localisation_imu/checkup_inertial_measurements.hpp"
#include "romea_core_localisation_imu/checkup_attitude.hpp"
#include "romea_core_localisation_imu/angular_speed_bias.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

class IMUPlugin
{
public:
  explicit IMUPlugin(std::unique_ptr<IMUAHRS> imu);

  void enable_debug_log(const std::string & logFilename);

  void process_linear_speed(
    const Duration & stamp,
    const double & linearSpeed);

  bool compute_angular_speed(
    const Duration & stamp,
    const double & acceleration_along_x_axis,
    const double & acceleration_along_y_axis,
    const double & acceleration_along_z_axis,
    const double & angular_speed_around_x_axis,
    const double & angular_speed_around_y_axis,
    const double & angular_speed_around_z_axis,
    ObservationAngularSpeed & angular_speed);

  bool compute_attitude(
    const Duration & stamp,
    const double & roll_angle,
    const double & pitch_angle,
    const double & course_angle,
    ObservationAttitude & attitude);

  DiagnosticReport make_diagnostic_report(const Duration & stamp);

private:
  void check_heart_beats_(const Duration & stamp);

  DiagnosticReport make_diagnostic_report_();

private:
  std::unique_ptr<IMUAHRS> imu_;
  AngularSpeedBias imu_angular_speed_bias_;
  std::atomic<double> linear_speed_;

  CheckupGreaterThanRate attitude_rate_diagnostic_;
  CheckupGreaterThanRate linear_speed_rate_diagnostic_;
  CheckupGreaterThanRate inertial_measurement_rate_diagnostic_;

  CheckupAttitude attitude_diagnostic_;
  CheckupInertialMeasurements inertial_measurement_diagnostic_;

  SimpleFileLogger debug_logger_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_IMU__LOCALISATIONIMUPLUGIN_HPP_
