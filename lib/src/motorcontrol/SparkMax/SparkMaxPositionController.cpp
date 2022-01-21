#include <algorithm>
#include <rmb/motorcontrol/SparkMax/SparkMaxError.h>
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

  sparkMax.RestoreFactoryDefaults();

  //configure pid consts
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetP(config.p));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetI(config.i));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetD(config.d));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetFF(config.f));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetIZone(config.iZone));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetIMaxAccum(config.iMaxAccumulator));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetOutputRange(config.minOutput));

  if(config.usingSmartMotion) {
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionAllowedClosedLoopError(
      RawUnit_t(config.allowedErr / conversion).to<double>()
    ));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionMaxVelocity(
      RawVelocity_t(config.maxVelocity / conversion).to<double>()
    ));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionMaxAccel(
      RawAccel_t(config.maxAccel / conversion).to<double>()
    ));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionAccelStrategy(config.accelStrategy));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionMinOutputVelocity(
      RawVelocity_t(config.minVelocity / conversion).to<double>()
    ));

    controlType = rev::ControlType::kSmartMotion;
  } else {
    controlType = rev::ControlType::kPosition;
  }
}

template <typename U>
void rmb::SparkMaxPositionController<U>::setPosition(Distance_t position) {
  double setPoint = RawUnit_t(position / conversion).to<double>;
  std::clamp<double>(setPoint, minPosition.to<double>(), maxPosition.to<double>());
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetReference(setPoint, controlType));
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
  CHECK_REVLIB_ERROR(sparkMaxEncoder.SetPosition(
    RawUnit_t(distance / conversion).to<double>()
  ));
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