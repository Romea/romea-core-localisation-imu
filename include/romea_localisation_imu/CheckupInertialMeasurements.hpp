#ifndef _romea_CheckupInertialMeasurements_hpp_
#define _romea_CheckupInertialMeasurements_hpp_

#include <romea_imu/AccelerationsFrame.hpp>
#include <romea_imu/AngularSpeedsFrame.hpp>
#include <romea_common/diagnostic/DiagnosticReport.hpp>

namespace romea
{

class CheckupInertialMeasurements
{
public :

  CheckupInertialMeasurements(const double & accelerationRange,
                              const double & angularSpeedRange);

  DiagnosticStatus evaluate(const AccelerationsFrame & accelerations,
                            const AngularSpeedsFrame & angularSpeeds);

  const DiagnosticReport & getReport() const;

private :

  void checkAccelerations_(const AccelerationsFrame & accelerationFrame);
  void checkAngularSpeeds_(const AngularSpeedsFrame & angularSpeedFrame);
  void setReportInfos_(const AccelerationsFrame & accelarations);
  void setReportInfos_(const AngularSpeedsFrame & angularSpeeds);
  void addDiagnostic_(const DiagnosticStatus & status,const std::string & message);

private :

  double accelerationRange_;
  double angularSpeedRange_;
  DiagnosticReport report_;
};

}

#endif
