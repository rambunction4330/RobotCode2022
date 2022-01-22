#include "rmb/motorcontrol/talon/TalonFXVelocityController.h"

#include <units/velocity.h>

namespace rmb {
template <typename U>
TalonFXVelocityController<U>::TalonFXVelocityController(int deviceNumber)
    : talonFX(deviceNumber) {
  talonFX.ConfigFactoryDefault();
  talonFX.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0);
}

template <typename U>
TalonFXVelocityController<U>::TalonFXVelocityController(
    int deviceNumber,
    const ctre::phoenix::motorcontrol::can::TalonFXConfiguration &config,
    ConversionUnit_t convert)
    : talonFX(deviceNumber), conversion(convert) {
  talonFX.ConfigFactoryDefault();
  talonFX.ConfigAllSettings(config);
  talonFX.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0);
}

template <typename U>
TalonFXVelocityController<U>::TalonFXVelocityController(
    int deviceNumber, const PIDConfig &config, ConversionUnit_t convert)
    : talonFX(deviceNumber), conversion(convert) {
  talonFX.ConfigFactoryDefault();
  talonFX.Config_kP(0, config.p);
  talonFX.Config_kI(0, config.i);
  talonFX.Config_kD(0, config.d);
  talonFX.Config_kF(0, config.f);
  talonFX.Config_IntegralZone(0, config.iZone);
  talonFX.ConfigMaxIntegralAccumulator(0, config.iMaxAccumulator);
  talonFX.ConfigAllowableClosedloopError(
      0, RawVelocity_t(config.allowableError / conversion).to<double>());
  talonFX.ConfigClosedLoopPeakOutput(0, config.maxOutput);
  talonFX.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0);
}

template <typename U>
void TalonFXVelocityController<U>::setVelocity(Velocity_t velocity) {
  double setPoint = RawVelocity_t(velocity / conversion).to<double>();
  talonFX.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
              setPoint);
}

template <typename U>
typename TalonFXVelocityController<U>::Velocity_t
TalonFXVelocityController<U>::getVelocity() {
  return RawVelocity_t(talonFX.GetSelectedSensorVelocity()) * conversion;
}

template class TalonFXVelocityController<units::meters>;
template class TalonFXVelocityController<units::radians>;
} // namespace rmb