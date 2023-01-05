// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_LOCALISATION_IMU__CHECKUPATTITUDE_HPP_
#define ROMEA_CORE_LOCALISATION_IMU__CHECKUPATTITUDE_HPP_

// romea
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>
#include <romea_core_imu/RollPitchCourseFrame.hpp>

// std
#include <mutex>
#include <string>


namespace romea
{

class CheckupAttitude
{
public:
  CheckupAttitude();

  DiagnosticStatus evaluate(const RollPitchCourseFrame & frame);

  DiagnosticReport getReport()const;

  void reset();

private:
  bool checkAttitudeAngles_(const RollPitchCourseFrame & frame);

  void declareReportInfos_();
  void setReportInfos_(const RollPitchCourseFrame & frame);
  void setDiagnostic_(const DiagnosticStatus & status, const std::string & message);

private:
  mutable std::mutex mutex_;
  DiagnosticReport report_;
};

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_IMU__CHECKUPATTITUDE_HPP_
