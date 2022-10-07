#ifndef _romea_CheckupAttitude_hpp_
#define _romea_CheckupAttitude_hpp_

//romea
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>
#include <romea_core_imu/RollPitchCourseFrame.hpp>

//std
#include <mutex>

namespace romea {

class  CheckupAttitude
{

public:

  CheckupAttitude();

  DiagnosticStatus evaluate(const RollPitchCourseFrame & frame);

  DiagnosticReport getReport()const;

  void reset();

private :

  bool checkAttitudeAngles_(const RollPitchCourseFrame & frame);

  void declareReportInfos_();
  void setReportInfos_(const RollPitchCourseFrame & frame);
  void setDiagnostic_(const DiagnosticStatus & status, const std::string & message);

private :

  mutable std::mutex mutex_;
  DiagnosticReport report_;
};



}// namespace


#endif
