#ifndef __GPSLocalisationPlugin2_HPP__
#define __GPSLocalisationPlugin2_HPP__

//std
#include <memory>

//romea
#include <romea_imu/IMUAHRS.hpp>
#include <romea_common/log/SimpleFileLogger.hpp>
#include <romea_common/diagnostic/CheckupRate.hpp>
#include <romea_localisation/ObservationAngularSpeed.hpp>
#include <romea_localisation/ObservationAttitude.hpp>
#include "romea_localisation_imu/CheckupInertialMeasurements.hpp"
#include "romea_localisation_imu/CheckupAttitude.hpp"
#include "romea_localisation_imu/AngularSpeedBias.hpp"

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

  DiagnosticReport makeDiagnosticReport();

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
