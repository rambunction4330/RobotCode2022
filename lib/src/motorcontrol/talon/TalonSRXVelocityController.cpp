#include "rmb/motorcontrol/talon/TalonSRXVelocityController.h"

#include <units/velocity.h>

namespace rmb {
template <typename U>
TalonSRXVelocityController<U>::TalonSRXVelocityController(int deviceNumber)
    : talonSRX(deviceNumber) {
  talonSRX.ConfigFactoryDefault();
  talonSRX.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative, 0);
}

template <typename U>
TalonSRXVelocityController<U>::TalonSRXVelocityController(
    int deviceNumber,
    const ctre::phoenix::motorcontrol::can::TalonSRXConfiguration &config,
    ConversionUnit_t convert)
    : talonSRX(deviceNumber), conversion(convert) {
  talonSRX.ConfigFactoryDefault();
  talonSRX.ConfigAllSettings(config);
  talonSRX.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative, 0);
}

template <typename U>
TalonSRXVelocityController<U>::TalonSRXVelocityController(
    int deviceNumber, const PIDConfig &config, ConversionUnit_t convert)
    : talonSRX(deviceNumber), conversion(convert) {
  talonSRX.ConfigFactoryDefault();
  talonSRX.Config_kP(0, config.p);
  talonSRX.Config_kI(0, config.i);
  talonSRX.Config_kD(0, config.d);
  talonSRX.Config_kF(0, config.f);
  talonSRX.Config_IntegralZone(0, config.iZone);
  talonSRX.ConfigMaxIntegralAccumulator(0, config.iMaxAccumulator);
  talonSRX.ConfigAllowableClosedloopError(
      0, RawVelocity_t(config.allowableError / conversion).to<double>());
  talonSRX.ConfigClosedLoopPeakOutput(0, config.maxOutput);
  talonSRX.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative, 0);
}

template <typename U>
void TalonSRXVelocityController<U>::setVelocity(Velocity_t velocity) {
  double setPoint = RawVelocity_t(velocity / conversion).to<double>();
  talonSRX.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Velocity,
               setPoint);
}

template <typename U>
typename TalonSRXVelocityController<U>::Velocity_t
TalonSRXVelocityController<U>::getVelocity() {
  return RawVelocity_t(talonSRX.GetSelectedSensorVelocity()) * conversion;
}

template class TalonSRXVelocityController<units::meters>;
template class TalonSRXVelocityController<units::radians>;
} // namespace rmb