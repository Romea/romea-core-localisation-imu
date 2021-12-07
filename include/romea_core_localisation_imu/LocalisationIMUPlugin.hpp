#ifndef __IMULocalisationPlugin2_HPP__
#define __IMULocalisationPlugin2_HPP__

//std
#include <memory>

//romea
#include <romea_core_imu/IMUAHRS.hpp>
#include <romea_core_common/log/SimpleFileLogger.hpp>
#include <romea_core_common/diagnostic/CheckupRate.hpp>
#include <romea_core_localisation/ObservationAngularSpeed.hpp>
#include <romea_core_localisation/ObservationAttitude.hpp>
#include "romea_core_localisation_imu/CheckupInertialMeasurements.hpp"
#include "romea_core_localisation_imu/CheckupAttitude.hpp"
#include "romea_core_localisation_imu/AngularSpeedBias.hpp"

namespace romea {


class LocalisationIMUPlugin
{

public :

  LocalisationIMUPlugin(std::unique_ptr<IMUAHRS> imu);

  void enableDebugLog(const std::string & logFilename);

  bool computeAngularSpeed(const Duration & stamp,
                           const double & linearSpeed,
                           const double & accelerationAlongXAxis,
                           const double & accelerationAlongYAxis,
                           const double & accelerationAlongZAxis,
                           const double & angularSpeedAroundXAxis,
                           const double & angularSpeedAroundYAxis,
                           const double & angularSpeedAroundZAxis,
                           ObservationAngularSpeed & angularSpeed);

  bool computeAttitude(const Duration & stamp,
                       const double & rollAngle,
                       const double & pitchAngle,
                       const double & courseAngle,
                       ObservationAttitude & attitude);

  DiagnosticReport makeDiagnosticReport()const;

private:

  std::unique_ptr<IMUAHRS> imu_;

  double angularSpeedBias_;
  AngularSpeedBias angularSpeedBiasEstimator_;

  CheckupRate inertialMeasurementRateDiagnostic_;
  CheckupRate attitudeRateDiagnostic_;
  CheckupInertialMeasurements inertialMeasurementDiagnostic_;
  CheckupAttitude attitudeDiagnostic_;

  SimpleFileLogger debugLogger_;
};

}

#endif
