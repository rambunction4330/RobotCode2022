#pragma once

#include <units/base.h>
#include <units/time.h>
#include <units/voltage.h>

namespace rmb {
template <typename DistanceUnit> class Feedforward {
public:
  using Distance_t = units::unit_t<DistanceUnit>;
  using VelocityUnit = units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>;
  using AccelerationUnit = units::compound_unit<VelocityUnit, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<AccelerationUnit>;

  using KsUnit = units::volts;
  using Ks_t = units::unit_t<KsUnit>;
  using KvUnit =  units::compound_unit<units::volts, units::inverse<VelocityUnit>>;
  using Kv_t = units::unit_t<KvUnit>;
  using KaUnit = units::compound_unit<units::volts, units::inverse<AccelerationUnit>>;
  using Ka_t = units::unit_t<KaUnit>;

  virtual units::volt_t calculate(Velocity_t velocity, Distance_t distance = Distance_t(0.0), Acceleration_t acceleration = Acceleration_t(0.0)) const = 0;

  virtual Velocity_t maxAchievableVelocity(units::volt_t maxVoltage, Acceleration_t acceleration, Distance_t position) const = 0;
  virtual Velocity_t minAchievableVelocity(units::volt_t maxVoltage, Acceleration_t acceleration, Distance_t position) const = 0;
  virtual Acceleration_t maxAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity, Distance_t position) const = 0;
  virtual Acceleration_t minAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity, Distance_t position) const = 0;

  virtual Kv_t getVelocityGain() const = 0;
  virtual Ka_t getAcclerationGain() const = 0;
  virtual units::volt_t calculateStatic(Velocity_t velocity, Distance_t position = Distance_t(0)) const = 0; 
};
} // namespace rmb
