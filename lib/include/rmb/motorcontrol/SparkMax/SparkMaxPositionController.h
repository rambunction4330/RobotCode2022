#pragma once

#include <units/angle.h>
#include <units/base.h>
#include <units/time.h>

#include <rev/CANSparkMax.h>

#include "rmb/motorcontrol/PositionController.h"
#include "rmb/motorcontrol/feedforward/SimpleMotorFeedforward.h" 

namespace rmb {
template <typename DistanceUnit>
class SparkMaxPositionController : PositionController<DistanceUnit> {
public:
  using Distance_t = typename PositionController<DistanceUnit>::Distance_t;

  using VeloctyUnit = typename PositionController<DistanceUnit>::VelocityUnit;
  using Velocity_t = typename PositionController<DistanceUnit>::Velocity_t;

  using AccelerationUnit =
      typename PositionController<DistanceUnit>::AccelerationUnit;
  using Acceleration_t =
      typename PositionController<DistanceUnit>::Acceleration_t;

  // position functions for the spark max take in rotations
  // 1 rotation * 2pi = 2pi rad
  using RawUnit =
      typename units::unit<std::ratio<2>, units::radians, std::ratio<1>>;
  using RawUnit_t = typename units::unit_t<RawUnit>;
  // velocity is in rpm
  using RawVelocity =
      typename units::compound_unit<RawUnit, units::inverse<units::minutes>>;
  using RawVelocity_t = typename units::unit_t<RawVelocity>;
  // acceleration is in rpm / sec, or at least that's what the documentation
  // says
  using RawAccel =
      typename units::compound_unit<RawVelocity,
                                    units::inverse<units::seconds>>;
  using RawAccel_t = typename units::unit_t<RawAccel>;

  // user defined conversion unit
  using ConversionUnit =
      typename units::compound_unit<DistanceUnit,
                                    units::inverse<units::radians>>;
  using ConversionUnit_t = typename units::unit_t<ConversionUnit>;

  struct PIDConfig {
    double p, i, d, f;
    double iZone, iMaxAccumulator;
    double maxOutput, minOutput;

    // SmartMotion config
    bool usingSmartMotion;
    Velocity_t maxVelocity, minVelocity;
    Acceleration_t maxAccel;
    Distance_t allowedErr;
    rev::SparkMaxPIDController::AccelStrategy accelStrategy;
  };

  struct Follower {
    int id;
    rev::CANSparkMax::MotorType motorType;
    bool inverted;
  };

  SparkMaxPositionController(int deviceID);
  SparkMaxPositionController(int deviceID, const PIDConfig &pidConfig,
                             ConversionUnit_t conversion = ConversionUnit_t(1), 
                             const Feedforward<DistanceUnit>& feedForward = noFeedforward<DistanceUnit>,
                             std::initializer_list<Follower> followers = {});

  void setPosition(Distance_t position) override;
  Distance_t getPosition() override;
  Velocity_t getVelocity() override;

  inline void setInverted(bool inverted) override {
    sparkMax.SetInverted(inverted);
  };
  inline bool getInverted() override { return sparkMax.GetInverted(); };

  void resetRefrence(Distance_t position) override;
  void setMaxPosition(Distance_t max) override;
  Distance_t getMaxPosition() override;
  void setMinPosition(Distance_t min) override;
  Distance_t getMinPosition() override;

private:
  rev::CANSparkMax sparkMax;
  rev::SparkMaxRelativeEncoder sparkMaxEncoder;
  rev::SparkMaxPIDController sparkMaxPIDController;

  ConversionUnit_t conversion;

  RawUnit_t maxPosition = RawUnit_t(1);
  RawUnit_t minPosition = RawUnit_t(-1);

  rev::CANSparkMax::ControlType controlType;
  std::vector<std::unique_ptr<rev::CANSparkMax>> followers;

  const Feedforward<DistanceUnit> &feedforward;
};
} // namespace rmb