#ifndef _romea_AngularSpeedBias_hpp_
#define _romea_AngularSpeedBias_hpp_

//romea
#include <romea_imu/algorithms/ZeroVelocityEstimator.hpp>
#include <romea_imu/AccelerationsFrame.hpp>
#include <romea_imu/AngularSpeedsFrame.hpp>
#include <romea_common/diagnostic/DiagnosticReport.hpp>

namespace romea {

class AngularSpeedBias
{

public:

  AngularSpeedBias(const double &imuRate,
                   const double & accelerationSpeedStd,
                   const double & angularSpeedStd);

  bool evaluate(const double &linearSpeed,
                const AccelerationsFrame & accelerations,
                const AngularSpeedsFrame & angularSpeeds,
                double & angularSpeedBias);

  const DiagnosticReport & getReport()const;

private :

  bool isStopped_(const double &linearSpeed,
                  const AccelerationsFrame &accelerations,
                  const AngularSpeedsFrame &angularSpeeds);

  void setDiagnostic_(const DiagnosticStatus & status,
                      const std::string & message);

private :

  bool is_stopped_;
  ZeroVelocityEstimator zeroVelocity_;
  romea::OnlineAverage imu_angular_speed_bias_;

  DiagnosticReport report_;
};


}// namespace


#endif
