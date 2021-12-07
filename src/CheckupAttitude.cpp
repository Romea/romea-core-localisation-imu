#include "romea_core_localisation_imu/CheckupAttitude.hpp"

namespace
{
}

namespace romea {

//-----------------------------------------------------------------------------
CheckupAttitude::CheckupAttitude():
  report_()
{
  report_.diagnostics.push_back(Diagnostic());
}

//-----------------------------------------------------------------------------
DiagnosticStatus CheckupAttitude::evaluate(const RollPitchCourseFrame & frame)
{
  if(checkAttitudeAngles_(frame))
  {
     setDiagnostic_(DiagnosticStatus::OK,"Attitude is OK.");
  }
  else
  {
    setDiagnostic_(DiagnosticStatus::ERROR,"Attitude angles are out of range.");
  }

  setReportInfos_(frame);
  return report_.diagnostics.front().status;
}


//-----------------------------------------------------------------------------
void CheckupAttitude::setDiagnostic_(const DiagnosticStatus & status,
                                     const std::string & message)
{
  Diagnostic & diagnostic = report_.diagnostics.front();
  diagnostic.message = message;
  diagnostic.status = status;
}

//-----------------------------------------------------------------------------
bool CheckupAttitude::checkAttitudeAngles_(const RollPitchCourseFrame & frame)
{
  return frame.rollAngle>=-M_PI_2 &&
      frame.rollAngle<= M_PI_2 &&
      frame.pitchAngle>=-M_PI_2 &&
      frame.pitchAngle<= M_PI_2;
}

//-----------------------------------------------------------------------------
void CheckupAttitude::setReportInfos_(const RollPitchCourseFrame & frame)
{
  setReportInfo(report_,"roll",frame.rollAngle);
  setReportInfo(report_,"pitch",frame.pitchAngle);
}

//-----------------------------------------------------------------------------
const DiagnosticReport & CheckupAttitude::getReport()const
{
  return report_;
}

}// namespace



