#include "romea_localisation_imu/LocalisationIMUPlugin.hpp"

namespace
{
const double LINEAR_SPEED_EPSILON = 0.001;
}


namespace romea {

//-----------------------------------------------------------------------------
LocalisationIMUPlugin::LocalisationIMUPlugin(std::unique_ptr<IMUAHRS> imu,
                                             const double & minimalRate):
  imu_(std::move(imu)),
  angularSpeedBias_(0),
  angularSpeedBiasEstimator_(imu_->getRate(),
                             imu_->getAccelerationStd(),
                             imu_->getAngularSpeedStd()),
  inertialMeasurementRateDiagnostic_("inertial_measurements",
                                     minimalRate,minimalRate*0.1),
  attitudeRateDiagnostic_("attitude",
                          minimalRate,minimalRate*0.1),
  inertialMeasurementDiagnostic_(imu_->getAccelerationRange(),
                                 imu_->getAngularSpeedRange()),
  attitudeDiagnostic_(),
  debugLogger_()
{

}

//-----------------------------------------------------------------------------
bool LocalisationIMUPlugin::computeAngularSpeed(const Duration & stamp,
                                                const double & linearSpeed,
                                                const double & accelerationAlongXAxis,
                                                const double & accelerationAlongYAxis,
                                                const double & accelerationAlongZAxis,
                                                const double & angularSpeedAroundXAxis,
                                                const double & angularSpeedAroundYAxis,
                                                const double & angularSpeedAroundZAxis,
                                                ObservationAngularSpeed & angularSpeed)
{

  romea::AccelerationsFrame accelerations =
      imu_->createAccelerationsFrame(accelerationAlongXAxis,
                                     accelerationAlongYAxis,
                                     accelerationAlongZAxis);


  romea::AngularSpeedsFrame angularSpeeds =
      imu_->createAngularSpeedsFrame(angularSpeedAroundXAxis,
                                     angularSpeedAroundYAxis,
                                     angularSpeedAroundZAxis);


  if(inertialMeasurementRateDiagnostic_.evaluate(stamp)==DiagnosticStatus::OK &&
     inertialMeasurementDiagnostic_.evaluate(accelerations,angularSpeeds)==DiagnosticStatus::OK)
  {

    if(angularSpeedBiasEstimator_.evaluate(linearSpeed,
                                           accelerations,
                                           angularSpeeds,
                                           angularSpeedBias_))

    {
      angularSpeed.Y()= angularSpeeds.angularSpeedAroundZAxis - angularSpeedBias_;
      angularSpeed.R()= imu_->getAngularSpeedVariance();
      return true;
    }
  }

  return false;
}

//-----------------------------------------------------------------------------
bool LocalisationIMUPlugin::computeAttitude(const Duration & stamp,
                                            const double & rollAngle,
                                            const double & pitchAngle,
                                            const double & courseAngle,
                                            ObservationAttitude & attitude)
{

  RollPitchCourseFrame frame= imu_->createFrame(rollAngle,
                                                pitchAngle,
                                                courseAngle);

  if(attitudeRateDiagnostic_.evaluate(stamp)==DiagnosticStatus::OK &&
     attitudeDiagnostic_.evaluate(frame)==DiagnosticStatus::OK)
  {
    attitude.Y(ObservationAttitude::ROLL)=rollAngle;
    attitude.Y(ObservationAttitude::PITCH)=pitchAngle;
    attitude.R()=Eigen::Matrix2d::Identity()*imu_->getAngleVariance();
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
DiagnosticReport LocalisationIMUPlugin::makeDiagnosticReport()
{
  DiagnosticReport report;
  report += inertialMeasurementRateDiagnostic_.getReport();
  report += attitudeRateDiagnostic_.getReport();
  report += inertialMeasurementDiagnostic_.getReport();
  report += attitudeDiagnostic_.getReport();
  report += angularSpeedBiasEstimator_.getReport();
  return report;
}


}

