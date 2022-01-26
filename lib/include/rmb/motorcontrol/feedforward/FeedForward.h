#pragma once

#include <units/base.h>
#include <units/time.h>
#include <units/voltage.h>

namespace rmb {
template <typename DistanceUnit> class Feedforward {
public:
  using Distance_t = units::unit_t<DistanceUnit>;
  using VelocityUnit =
      units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>;
  using AccelerationUnit =
      units::compound_unit<VelocityUnit, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<AccelerationUnit>;

  virtual units::volt_t
  calculate(Velocity_t velocity, Distance_t distance = Distance_t(0.0),
            Acceleration_t acceleration = Acceleration_t(0.0)) const = 0;
};
} // namespace rmb