#pragma once

#include <units/base.h>
#include <units/angle.h>

#include <rev/CANSparkMax.h>
#include "rmb/motorcontrol/VelocityController.h"

#include <frc/MotorSafety.h>

namespace rmb {
  // an abstraction over the SparkMax motor controller
  template <typename DistanceUnit>
  class SparkMaxVelocityController : public VelocityController<DistanceUnit> {
  public:

    using Distance_t   = typename VelocityController<DistanceUnit>::Distance_t;

    using VeloctyUnit = typename VelocityController<DistanceUnit>::VelocityUnit;
    using Velocity_t  = typename VelocityController<DistanceUnit>::Velocity_t;

    using AccelerationUnit = typename VelocityController<DistanceUnit>::AccelerationUnit;
    using Acceleration_t   = typename VelocityController<DistanceUnit>::Acceleration_t;

    // user defined conversion factor
    using ConversionUnit   = typename units::compound_unit<DistanceUnit, units::inverse<units::radians>>;
    using ConversionUnit_t = typename units::unit_t<ConversionUnit>;

    // raw velocity is in rpm. 1 rotation * 2pi rad = 2pi rad
    using RawUnit   = typename units::unit<std::ratio<2>, units::radians, std::ratio<1>>;
    using RawUnit_t = typename units::unit_t<RawUnit>;
    using RawVelocity   = typename units::compound_unit<RawUnit, units::inverse<units::seconds>>;
    using RawVelocity_t = typename units::unit_t<RawVelocity>;
    
    struct PIDConfig {
      double p, i, d, f;
      double iZone, iMaxAccumulator;
      Velocity_t allowableError;
      double maxOutput, minOutput;
    };

    SparkMaxVelocityController(int deviceID);
    SparkMaxVelocityController(int deviceID, const PIDConfig& config, ConversionUnit_t conversionUnit);

    void setVelocity(Velocity_t velocity) override;
    Velocity_t getVelocity() override;

    inline void setInverted(bool inverted) override { sparkMax.SetInverted(inverted) };
    inline bool getInverted() override { return sparkMax.GetInverted() };
    inline void disable() override { sparkMax.Disable(); }
    inline void stopMotor() override { sparkMax.StopMotor(); };

  private:

    rev::CANSparkMax sparkMax;
    rev::SparkMaxRelativeEncoder sparkMaxEncoder;
    rev::SparkMaxPIDController sparkMaxPIDController;
    ConversionUnit_t conversion;
  };
} // namespace rmb