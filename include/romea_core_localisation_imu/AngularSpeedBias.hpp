#ifndef _romea_AngularSpeedBias_hpp_
#define _romea_AngularSpeedBias_hpp_

//romea
#include <romea_core_imu/algorithms/ZeroVelocityEstimator.hpp>
#include <romea_core_imu/AccelerationsFrame.hpp>
#include <romea_core_imu/AngularSpeedsFrame.hpp>
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

//std
#include <optional>
#include <mutex>

namespace romea {

class AngularSpeedBias
{

public:

  AngularSpeedBias(const double &imuRate,
                   const double & accelerationSpeedStd,
                   const double & angularSpeedStd);

  std::optional<double> evaluate(const double & linearSpeed,
                                 const AccelerationsFrame & accelerations,
                                 const AngularSpeedsFrame & angularSpeeds);

  DiagnosticReport getReport()const;

  void reset(bool resetZeroVelocityEstimator);

private :

  bool hasNullLinearSpeed_(const double & linearSpeed)const;

  bool hasZeroVelocity_(const AccelerationsFrame & accelerations,
                        const AngularSpeedsFrame & angularSpeeds);

  void updateAngularSpeedBias_(const double & linearSpeed,
                               const AccelerationsFrame & accelerations,
                               const AngularSpeedsFrame & angularSpeeds);

  void setDiagnostic_(const DiagnosticStatus & status,const std::string & message);

private :

  ZeroVelocityEstimator zeroVelocity_;
  romea::OnlineAverage imuAngularSpeedBiasEstimator_;

  mutable std::mutex mutex_;
  DiagnosticReport report_;
};


}// namespace


#endif
