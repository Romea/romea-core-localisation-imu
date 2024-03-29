#ifndef ROMEA_CORE_LOCALISATION_IMU__CHECKUPINERTIALMEASUREMENTS_HPP_
#define ROMEA_CORE_LOCALISATION_IMU__CHECKUPINERTIALMEASUREMENTS_HPP_


// romea
#include <romea_core_imu/AccelerationsFrame.hpp>
#include <romea_core_imu/AngularSpeedsFrame.hpp>
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

// std
#include <mutex>
#include <string>

namespace romea
{
namespace core
{

class CheckupInertialMeasurements
{
public:
  CheckupInertialMeasurements(
    const double & accelerationRange,
    const double & angularSpeedRange);

  DiagnosticStatus evaluate(
    const AccelerationsFrame & accelerations,
    const AngularSpeedsFrame & angularSpeeds);

  DiagnosticReport getReport() const;

  void reset();

private:
  void checkAccelerations_(const AccelerationsFrame & accelerationFrame);
  void checkAngularSpeeds_(const AngularSpeedsFrame & angularSpeedFrame);

  void declareReportInfos_();
  void setReportInfos_(const AccelerationsFrame & accelarations);
  void setReportInfos_(const AngularSpeedsFrame & angularSpeeds);
  void addDiagnostic_(const DiagnosticStatus & status, const std::string & message);

private:
  double accelerationRange_;
  double angularSpeedRange_;

  mutable std::mutex mutex_;
  DiagnosticReport report_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION_IMU__CHECKUPINERTIALMEASUREMENTS_HPP_
