#include "romea_core_localisation_imu/CheckupInertialMeasurements.hpp"
#include <iostream>

namespace romea
{


//-----------------------------------------------------------------------------
CheckupInertialMeasurements::CheckupInertialMeasurements(const double &accelerationRange,
                                                         const double & angularSpeedRange):
  accelerationRange_(accelerationRange),
  angularSpeedRange_(angularSpeedRange),
  report_()
{
}

//-----------------------------------------------------------------------------
DiagnosticStatus CheckupInertialMeasurements::evaluate(const AccelerationsFrame & accelerations,
                                                       const AngularSpeedsFrame & angularSpeeds)
{
  report_.diagnostics.clear();
  checkAccelerations_(accelerations);
  checkAngularSpeeds_(angularSpeeds);
  setReportInfos_(accelerations);
  setReportInfos_(angularSpeeds);
  return worseStatus(report_.diagnostics);
}

//--------------------------------------------------------------------
void CheckupInertialMeasurements::checkAccelerations_(const AccelerationsFrame & accelerationFrame)
{
  if(std::abs(accelerationFrame.accelerationAlongXAxis)>accelerationRange_||
      std::abs(accelerationFrame.accelerationAlongYAxis)>accelerationRange_ ||
      std::abs(accelerationFrame.accelerationAlongZAxis)>accelerationRange_)
  {
    addDiagnostic_(DiagnosticStatus::ERROR,"Acceleration data is out of range.");
  }
  else
  {
    addDiagnostic_(DiagnosticStatus::OK,"Acceleration data is OK.");
  }
}

//--------------------------------------------------------------------
void CheckupInertialMeasurements::checkAngularSpeeds_(const AngularSpeedsFrame & angularSpeedFrame)
{
  if(std::abs(angularSpeedFrame.angularSpeedAroundXAxis)>angularSpeedRange_||
      std::abs(angularSpeedFrame.angularSpeedAroundYAxis)>angularSpeedRange_ ||
      std::abs(angularSpeedFrame.angularSpeedAroundZAxis)>angularSpeedRange_)
  {
    addDiagnostic_(DiagnosticStatus::ERROR,"Angular speed data is out of range.");
  }
  else
  {
    addDiagnostic_(DiagnosticStatus::OK,"Angular speed data is OK.");
  }
}


//-----------------------------------------------------------------------------
const DiagnosticReport & CheckupInertialMeasurements::getReport() const
{
  return report_;
}


//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::setReportInfos_(const AccelerationsFrame & accelarations)
{
  setReportInfo(report_,"acceleration_x",accelarations.accelerationAlongXAxis);
  setReportInfo(report_,"acceleration_y",accelarations.accelerationAlongYAxis);
  setReportInfo(report_,"acceleration_z",accelarations.accelerationAlongZAxis);
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::setReportInfos_(const AngularSpeedsFrame & angularSpeeds)
{
  setReportInfo(report_,"angular_speed_x",angularSpeeds.angularSpeedAroundXAxis);
  setReportInfo(report_,"angular_speed_y",angularSpeeds.angularSpeedAroundYAxis);
  setReportInfo(report_,"angular_speed_z",angularSpeeds.angularSpeedAroundZAxis);
}

//-----------------------------------------------------------------------------
void CheckupInertialMeasurements::addDiagnostic_(const DiagnosticStatus & status,
                                                 const std::string & message)
{
  Diagnostic diagnostic;
  diagnostic.message = message;
  diagnostic.status = status;
  report_.diagnostics.push_back(diagnostic);
}

}
