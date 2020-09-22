#ifndef _romea_CheckupAttitude_hpp_
#define _romea_CheckupAttitude_hpp_

//romea
#include <romea_common/diagnostic/DiagnosticReport.hpp>
#include <romea_imu/RollPitchCourseFrame.hpp>

namespace romea {

class  CheckupAttitude
{

public:

  CheckupAttitude();

  DiagnosticStatus evaluate(const RollPitchCourseFrame & frame);

  const DiagnosticReport & getReport()const;

private :

  bool checkAttitudeAngles_(const RollPitchCourseFrame & frame);
  void setReportInfos_(const RollPitchCourseFrame & frame);
  void setDiagnostic_(const DiagnosticStatus & status, const std::string & message);

private :

  DiagnosticReport report_;
};



}// namespace


#endif
