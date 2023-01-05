// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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

}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_IMU__LOCALISATIONIMUPLUGIN_HPP_ 
