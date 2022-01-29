#pragma once

#include <units/angle.h>
#include <units/base.h>

#include <rev/CANSparkMax.h>

#include "rmb/motorcontrol/VelocityController.h"
#include "rmb/motorcontrol/feedforward/SimpleMotorFeedforward.h"

namespace rmb {
// an abstraction over the SparkMax motor controller
template <typename DistanceUnit>
class SparkMaxVelocityController : public VelocityController<DistanceUnit> {
public:
  using Distance_t = typename VelocityController<DistanceUnit>::Distance_t;

  using VeloctyUnit = typename VelocityController<DistanceUnit>::VelocityUnit;
  using Velocity_t = typename VelocityController<DistanceUnit>::Velocity_t;

  using AccelerationUnit =
      typename VelocityController<DistanceUnit>::AccelerationUnit;
  using Acceleration_t =
      typename VelocityController<DistanceUnit>::Acceleration_t;

  // user defined conversion factor
  using ConversionUnit =
      typename units::compound_unit<DistanceUnit,
                                    units::inverse<units::radians>>;
  using ConversionUnit_t = typename units::unit_t<ConversionUnit>;

  // raw velocity is in rpm. 1 rotation * 2pi rad = 2pi rad
  using RawUnit =
      typename units::unit<std::ratio<2>, units::radians, std::ratio<1>>;
  using RawUnit_t = typename units::unit_t<RawUnit>;
  using RawVelocity =
      typename units::compound_unit<RawUnit, units::inverse<units::minutes>>;
  using RawVelocity_t = typename units::unit_t<RawVelocity>;
  using RawAccel =
      typename units::compound_unit<RawVelocity,
                                    units::inverse<units::seconds>>;
  using RawAccel_t = typename units::unit_t<RawAccel>;

  struct PIDConfig {
    double p = 0.0057181, i = 0.0, d = 0.0, f = 0.0;
    double iZone = 0.0, iMaxAccumulator = 0.0;
    double maxOutput = 1.0, minOutput = -1.0;

    // SmartMotion config
    bool usingSmartMotion = true;
    Velocity_t maxVelocity = Velocity_t(25), minVelocity = Velocity_t(0);
    Acceleration_t maxAccel = Acceleration_t(10);
    Velocity_t allowedErr = Velocity_t(0.9);
    rev::SparkMaxPIDController::AccelStrategy accelStrategy =
        rev::SparkMaxPIDController::AccelStrategy::kSCurve;
  };

  struct Follower {
    int id;
    rev::CANSparkMax::MotorType motorType;
    bool inverted;
  };

  SparkMaxVelocityController(int deviceID);
  SparkMaxVelocityController(
      int deviceID, const PIDConfig &config,
      ConversionUnit_t conversionUnit = ConversionUnit_t(1),
      const Feedforward<DistanceUnit> &feedforward = noFeedforward<DistanceUnit>,
      std::initializer_list<Follower> followers = {});


  void setVelocity(Velocity_t velocity) override;
  Velocity_t getVelocity() override;

  inline void setInverted(bool inverted) override {
    sparkMax.SetInverted(inverted);
  };
  inline bool getInverted() override { return sparkMax.GetInverted(); };

private:
  rev::CANSparkMax sparkMax;
  rev::SparkMaxRelativeEncoder sparkMaxEncoder;
  rev::SparkMaxPIDController sparkMaxPIDController;
  ConversionUnit_t conversion;
  const Feedforward<DistanceUnit> &feedforward;

  rev::CANSparkMax::ControlType controlType;
  std::vector<std::unique_ptr<rev::CANSparkMax>> followers;
};
} // namespace rmb
