#include <rmb/motorcontrol/SparkMax/SparkMaxPositionController.h>

template <typename U>
rmb::SparkMaxPositionController<U>::SparkMaxPositionController(int deviceID) {
  sparkMax = rev::CANSparkMax{deviceID, rev::CANSparkMax::MotorType::kBrushless};
  sparkMaxEncoder = sparkMax.GetEncoder();
  sparkMaxPIDController = sparkMax.GetPIDController();

}

template <typename U>
rmb::SparkMaxPositionController<U>::SparkMaxPositionController(
                                                              int deviceID, 
                                                              const PIDConfig& config, 
                                                              ConversionUnit_t conversionFactor
                                                              ) : conversion(conversionFactor) {
  
  sparkMax = rev::CANSparkMax{deviceID, rev::CANSparkMax::MotorType::kBrushless};
  sparkMaxEncoder = sparkMax.GetEncoder();
  sparkMaxPIDController = sparkMax.GetPIDController();

  //configure pid consts
  sparkMaxPIDController.SetP(config.p);
  sparkMaxPIDController.SetI(config.i);
  sparkMaxPIDController.SetD(config.d);
  sparkMaxPIDController.SetFF(config.f);
  sparkMaxPIDController.SetIZone(config.iZone);
  sparkMaxPIDController.SetIMaxAccum(config.iMaxAccumulator);
  sparkMaxPIDController.SetOutputRange(config.minOutput);
}

template <typename U>
void rmb::SparkMaxPositionController<U>::setPosition(Distance_t position) {
  double setPoint = RawUnit_t(position / conversion).to<double>;
  sparkMaxPIDController.SetReference(setPoint, rev::ControlType::kPosition);
}

template <typename U>
typename rmb::SparkMaxPositionController<U>::Distance_t rmb::SparkMaxPositionController<U>::getPosition() {
  return Distance_t(RawUnit_t(sparkMaxEncoder.GetPosition()) * conversion);
}

template <typename U>
typename rmb::SparkMaxPositionController<U>::Velocity_t rmb::SparkMaxPositionController<U>::getVelocity() {
  return Velocity_t(RawVelocity_t(sparkMaxEncoder.GetVelocity()) * conversion);
}