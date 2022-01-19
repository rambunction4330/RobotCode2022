#include <algorithm>
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
  std::clamp<double>(setPoint, minPosition.to<double>(), maxPosition.to<double>());
  sparkMaxPIDController.SetReference(setPoint, rev::ControlType::kPosition);
}

template <typename U>
typename rmb::SparkMaxPositionController<U>::Distance_t rmb::SparkMaxPositionController<U>::getPosition() {
  RawUnit_t val = RawUnit_t(sparkMaxEncoder.GetPosition());
  std::clamp<RawUnit_t>(val, minPosition, maxPosition);
  return Velocity_t(val * conversion);
}

template <typename U>
typename rmb::SparkMaxPositionController<U>::Velocity_t rmb::SparkMaxPositionController<U>::getVelocity() {
  return Velocity_t(RawVelocity_t(sparkMaxEncoder.GetVelocity()) * conversion);
}

template <typename U>
void rmb::SparkMaxPositionController<U>::resetRefrence(Distance_t distance) {
  sparkMaxEncoder.SetPosition(RawUnit_t(distance / conversion).to<double>());
}

template <typename U>
void rmb::SparkMaxPositionController<U>::setMaxPosition(Distance_t max) {
  maxPosition = RawUnit_t(max / conversion);
}

template <typename U>
typename rmb::SparkMaxPositionController<U>::Distance_t rmb::SparkMaxPositionController<U>::getMaxPosition() {
  return Distance_t(maxPosition * conversion);
}

template <typename U>
void rmb::SparkMaxPositionController<U>::setMinPosition(Distance_t min) {
  minPosition = RawUnit_t(min / conversion);
}

template <typename U>
typename rmb::SparkMaxPositionController<U>::Distance_t rmb::SparkMaxPositionController<U>::getMinPosition() {
  return Distance_t(minPosition * conversion);  
}