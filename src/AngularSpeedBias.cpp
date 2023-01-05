// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// local
#include "romea_core_localisation_imu/AngularSpeedBias.hpp"

namespace
{
const double ANGULAR_SPEED_BIAS_EPSILON = 0.0001;
const double LINEAR_SPEED_EPSILON = 0.02;
}

namespace romea
{

//-----------------------------------------------------------------------------
AngularSpeedBias::AngularSpeedBias(
  const double & imuRate,
  const double & accelerationSpeedStd,
  const double & angularSpeedStd)
: zeroVelocity_(imuRate, accelerationSpeedStd, angularSpeedStd),
  imuAngularSpeedBiasEstimator_(ANGULAR_SPEED_BIAS_EPSILON, 5 * imuRate),
  mutex_(),
  report_()
{
  setReportInfo(report_, "acceleration_std", "");
  setReportInfo(report_, "angular_speed_std", "");
  setReportInfo(report_, "linear_speed", "");
  setReportInfo(report_, "angular_speed_bias", "");
}

//-----------------------------------------------------------------------------
bool AngularSpeedBias::hasNullLinearSpeed_(const double & linearSpeed)const
{
  return std::isfinite(linearSpeed) && std::abs(linearSpeed) < LINEAR_SPEED_EPSILON;
}

//-----------------------------------------------------------------------------
bool AngularSpeedBias::hasZeroVelocity_(
  const AccelerationsFrame & accelerations,
  const AngularSpeedsFrame & angularSpeeds)
{
  return zeroVelocity_.update(
    accelerations.accelerationAlongXAxis,
    accelerations.accelerationAlongYAxis,
    accelerations.accelerationAlongZAxis,
    angularSpeeds.angularSpeedAroundXAxis,
    angularSpeeds.angularSpeedAroundYAxis,
    angularSpeeds.angularSpeedAroundZAxis);
}


//-----------------------------------------------------------------------------
void AngularSpeedBias::updateAngularSpeedBias_(
  const double & linearSpeed,
  const AccelerationsFrame & accelerations,
  const AngularSpeedsFrame & angularSpeeds)
{
  bool hasNullLinearSpeed = hasNullLinearSpeed_(linearSpeed);
  bool hasZeroVelocity = hasZeroVelocity_(accelerations, angularSpeeds);

  if (hasZeroVelocity && hasNullLinearSpeed) {
    imuAngularSpeedBiasEstimator_.update(angularSpeeds.angularSpeedAroundZAxis);
  }
}


//-----------------------------------------------------------------------------
std::optional<double> AngularSpeedBias::evaluate(
  const double & linearSpeed,
  const AccelerationsFrame & accelerations,
  const AngularSpeedsFrame & angularSpeeds)
{
  updateAngularSpeedBias_(linearSpeed, accelerations, angularSpeeds);

  std::lock_guard<std::mutex> lock(mutex_);
  setReportInfo(report_, "acceleration_std", zeroVelocity_.getAccelerationStd());
  setReportInfo(report_, "angular_speed_std", zeroVelocity_.getAngularSpeedStd());
  setReportInfo(
    report_, "linear_speed", std::isfinite(linearSpeed) ? std::to_string(
      linearSpeed) : "");

  if (imuAngularSpeedBiasEstimator_.isAvailable()) {
    double angularSpeedBias = imuAngularSpeedBiasEstimator_.getAverage();
    setDiagnostic_(DiagnosticStatus::OK, "Angular speed bias is OK.");
    setReportInfo(report_, "angular_speed_bias", angularSpeedBias);
    return angularSpeedBias;
  } else {
    setDiagnostic_(DiagnosticStatus::WARN, "Angular speed bias not available.");
    setReportInfo(report_, "angular_speed_bias", "");
    return std::nullopt;
  }
}

//-----------------------------------------------------------------------------
void AngularSpeedBias::reset(bool resetZeroVelocityEstimator)
{
  std::lock_guard<std::mutex> lock(mutex_);
  report_.diagnostics.clear();

  if (resetZeroVelocityEstimator) {
    zeroVelocity_.reset();
    setReportInfo(report_, "acceleration_std", "");
    setReportInfo(report_, "angular_speed_std", "");
  } else {
    setReportInfo(report_, "linear_speed", "");
  }

  imuAngularSpeedBiasEstimator_.reset();
  setReportInfo(report_, "angular_speed_bias", "");
}

//-----------------------------------------------------------------------------
void AngularSpeedBias::setDiagnostic_(
  const DiagnosticStatus & status,
  const std::string & message)
{
  report_.diagnostics.clear();
  report_.diagnostics.push_back({status, message});
}

//-----------------------------------------------------------------------------
DiagnosticReport AngularSpeedBias::getReport()const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return report_;
}

}  // namespace romea
