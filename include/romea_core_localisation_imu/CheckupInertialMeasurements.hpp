#ifndef _romea_CheckupInertialMeasurements_hpp_
#define _romea_CheckupInertialMeasurements_hpp_

//romea
#include <romea_core_imu/AccelerationsFrame.hpp>
#include <romea_core_imu/AngularSpeedsFrame.hpp>
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

//std
#include <mutex>

namespace romea
{

class CheckupInertialMeasurements
{
public :

  CheckupInertialMeasurements(const double & accelerationRange,
                              const double & angularSpeedRange);

  DiagnosticStatus evaluate(const AccelerationsFrame & accelerations,
                            const AngularSpeedsFrame & angularSpeeds);

  DiagnosticReport getReport() const;

  void reset();

private :

  void checkAccelerations_(const AccelerationsFrame & accelerationFrame);
  void checkAngularSpeeds_(const AngularSpeedsFrame & angularSpeedFrame);

  void declareReportInfos_();
  void setReportInfos_(const AccelerationsFrame & accelarations);
  void setReportInfos_(const AngularSpeedsFrame & angularSpeeds);
  void addDiagnostic_(const DiagnosticStatus & status,const std::string & message);

private :

  double accelerationRange_;
  double angularSpeedRange_;

  mutable std::mutex mutex_;
  DiagnosticReport report_;
};

}

#endif
