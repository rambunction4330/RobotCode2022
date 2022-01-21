#pragma once

#include <units/base.h>
#include <units/time.h>

namespace rmb {
  template<typename DistanceUnit>
  class VelocityController {
   public:
    using Distance_t = units::unit_t<DistanceUnit>;
    using VelocityUnit = units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
    using Velocity_t = units::unit_t<VelocityUnit>;
    using AccelerationUnit = units::compound_unit<VelocityUnit, units::inverse< units::seconds>>;
    using Acceleration_t = units::unit_t<AccelerationUnit>;

    virtual void setVelocity(Velocity_t velocity) = 0;
    virtual Velocity_t getVelocity() const = 0;

    virtual void setInverted(bool inverted) = 0;
    virtual bool getInverted() const = 0;
  };
} // rmb namespace
