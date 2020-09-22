#include "romea_localisation_imu/AngularSpeedBias.hpp"

namespace
{
const double ANGULAR_SPEED_BIAS_EPSILON=0.0001;
const double LINEAR_SPEED_EPSILON=0.001;
}

namespace romea {

//-----------------------------------------------------------------------------
AngularSpeedBias::AngularSpeedBias(const double &imuRate,
                                   const double & accelerationSpeedStd,
                                   const double & angularSpeedStd):
  is_stopped_(false),
  zeroVelocity_(imuRate,accelerationSpeedStd,angularSpeedStd),
  imu_angular_speed_bias_(ANGULAR_SPEED_BIAS_EPSILON,5*imuRate),
  report_()
{
  report_.diagnostics.push_back(Diagnostic());
}

//-----------------------------------------------------------------------------
bool AngularSpeedBias::isStopped_(const double &linearSpeed,
                                  const AccelerationsFrame & accelerations,
                                  const AngularSpeedsFrame & angularSpeeds)
{
  is_stopped_=zeroVelocity_.update(accelerations.accelerationAlongXAxis,
                                   accelerations.accelerationAlongYAxis,
                                   accelerations.accelerationAlongZAxis,
                                   angularSpeeds.angularSpeedAroundXAxis,
                                   angularSpeeds.angularSpeedAroundYAxis,
                                   angularSpeeds.angularSpeedAroundZAxis) &&
      std::abs(linearSpeed) < LINEAR_SPEED_EPSILON;

  setReportInfo(report_,"is_stopped",is_stopped_);
  setReportInfo(report_,"acceleration_std",zeroVelocity_.getAccelerationStd());
  setReportInfo(report_,"angular_speed_std",zeroVelocity_.getAngularSpeedStd());

  return is_stopped_;
}


//-----------------------------------------------------------------------------
bool AngularSpeedBias::evaluate(const double &linearSpeed,
                                const AccelerationsFrame & accelerations,
                                const AngularSpeedsFrame & angularSpeeds,
                                double & angularSpeedBias)
{

  if(isStopped_(linearSpeed,accelerations,angularSpeeds))
  {
    imu_angular_speed_bias_.update(angularSpeeds.angularSpeedAroundZAxis);
  }

  if(imu_angular_speed_bias_.isAvailable())
  {
    angularSpeedBias= imu_angular_speed_bias_.getAverage();
    setDiagnostic_(DiagnosticStatus::OK,"Angular speed bias is OK.");
    setReportInfo(report_,"angular_speed_bias",imu_angular_speed_bias_.getAverage());
    return true;
  }
  else
  {
    setDiagnostic_(DiagnosticStatus::WARN,"Angular speed bias not available.");
    setReportInfo(report_,"angular_speed_bias","");
    return false;
  }

}


//-----------------------------------------------------------------------------
void AngularSpeedBias::setDiagnostic_(const DiagnosticStatus & status, const std::string & message)
{
  Diagnostic & diagnostic = report_.diagnostics.front();
  diagnostic.message = message;
  diagnostic.status = status;
}

//-----------------------------------------------------------------------------
const DiagnosticReport & AngularSpeedBias::getReport()const
{
  return report_;
}


}



