#pragma once

#include <units/base.h>

namespace rmb {
template <typename DistanceUnit> class PositionController {
public:
  using Distance_t = units::unit_t<DistanceUnit>;
  using VelocityUnit =
      units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>;
  using AccelerationUnit =
      units::compound_unit<VelocityUnit, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<AccelerationUnit>;

  virtual void resetRefrence(Distance_t position) = 0;
  virtual void setMaxPosition(Distance_t max) = 0;
  virtual Distance_t getMaxPosition() = 0;
  virtual void setMinPosition(Distance_t min) = 0;
  virtual Distance_t getMinPosition() = 0;

  virtual void setPosition(Distance_t position) = 0;
  virtual Distance_t getPosition() = 0;
  virtual Velocity_t getVelocity() = 0;

  virtual void setInverted(bool inverted) = 0;
  virtual bool getInverted() = 0;
};
} // namespace rmb
