#include "rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h"

#include <units/angle.h>
#include <units/length.h>

#include "rmb/motorcontrol/sparkmax/SparkMaxError.h"

namespace rmb {

template <typename U>
SparkMaxVelocityController<U>::SparkMaxVelocityController(int deviceID)
    : sparkMax(deviceID, rev::CANSparkMax::MotorType::kBrushless),
      sparkMaxEncoder(std::make_unique<rev::SparkMaxRelativeEncoder>(sparkMax.GetEncoder())),
      sparkMaxPIDController(sparkMax.GetPIDController()),
      feedforward(noFeedforward<U>) {}

template <typename U>
SparkMaxVelocityController<U>::SparkMaxVelocityController(
    int deviceID, const PIDConfig &config, ConversionUnit_t conversionUnit,
    const Feedforward<U> &ff, std::initializer_list<Follower> followerList, 
    bool alternateEncoder, int countsPerRevolution, rev::CANSparkMax::MotorType motorType)
    : sparkMax(deviceID, motorType),
      sparkMaxEncoder(
        alternateEncoder ? 
          std::unique_ptr<rev::RelativeEncoder>(std::make_unique<rev::SparkMaxAlternateEncoder>(sparkMax.GetAlternateEncoder(countsPerRevolution))) :
          std::unique_ptr<rev::RelativeEncoder>(std::make_unique<rev::SparkMaxRelativeEncoder>(sparkMax.GetEncoder()))
        ),
      sparkMaxPIDController(sparkMax.GetPIDController()),
      conversion(conversionUnit), feedforward(ff) {

  CHECK_REVLIB_ERROR(sparkMax.RestoreFactoryDefaults());

  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetFeedbackDevice(*sparkMaxEncoder));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetP(config.p));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetI(config.i));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetD(config.d));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetFF(config.f));
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetIZone(config.iZone));
  CHECK_REVLIB_ERROR(
      sparkMaxPIDController.SetIMaxAccum(config.iMaxAccumulator));
  CHECK_REVLIB_ERROR(
      sparkMaxPIDController.SetOutputRange(config.minOutput, config.maxOutput));

  if (config.usingSmartMotion) {
    CHECK_REVLIB_ERROR(
        sparkMaxPIDController.SetSmartMotionAllowedClosedLoopError(
            RawVelocity_t(config.allowedErr / conversion).to<double>()));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionMaxVelocity(
        RawVelocity_t(config.maxVelocity / conversion).to<double>()));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionMaxAccel(
        RawAccel_t(config.maxAccel / conversion).to<double>()));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionAccelStrategy(
        config.accelStrategy));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionMinOutputVelocity(
        RawVelocity_t(config.minVelocity / conversion).to<double>()));

    controlType = rev::CANSparkMax::ControlType::kSmartVelocity;
  } else {
    controlType = rev::CANSparkMax::ControlType::kVelocity;
  }

  if (&feedforward != &noFeedforward<U>) {
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetFF(
        units::unit_t<units::inverse<RawVelocity>>(
            feedforward.getVelocityGain() * conversion / 12_V)
            .to<double>()));
  }

  followers.reserve(followerList.size());
  for (const auto &follower : followerList) {
    followers.emplace_back(
        new rev::CANSparkMax(follower.id, follower.motorType));
    followers.back()->Follow(sparkMax, follower.inverted);
  }
}

template <typename U>
void SparkMaxVelocityController<U>::setVelocity(Velocity_t velocity) {
  double setPoint = RawVelocity_t(velocity / conversion).to<double>();
  CHECK_REVLIB_ERROR(sparkMaxPIDController.SetReference(
      setPoint, controlType, 0,
      units::volt_t(feedforward.calculateStatic(velocity)).to<double>()));
}

template <typename U>
typename SparkMaxVelocityController<U>::Velocity_t
SparkMaxVelocityController<U>::getVelocity() {
  return Velocity_t(RawVelocity_t(sparkMaxEncoder -> GetVelocity()) * conversion);
}

template class SparkMaxVelocityController<units::meters>;
template class SparkMaxVelocityController<units::radians>;

} // namespace rmb
