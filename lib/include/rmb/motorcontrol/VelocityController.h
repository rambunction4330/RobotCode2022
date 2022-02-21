#pragma once

#include <units/base.h>
#include <units/time.h>

namespace rmb {
  
/**
* The base class for wrappers around classes that abstract over device-specific apis to control the
* velocity at which a motor spins.
*/
template <typename DistanceUnit> class VelocityController {
public:
  using Distance_t = units::unit_t<DistanceUnit>; /**< User specified distance unit*/
  using VelocityUnit =
      units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>; /**< User specificed distance unit / second*/
  using AccelerationUnit =
      units::compound_unit<VelocityUnit, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<AccelerationUnit>; /**< User specified distance unit / second^2*/

  /**
   * Sets the target velocity of the motor.
   * @param velocity the target velocity in user specified velocity units
   * @see Velocity_t
   */
  virtual void setVelocity(Velocity_t velocity) = 0;
  
  /**
   * Get the <b>current</b> velocity of the motor
   * @return the velocity of the motor in user specified velocity units
   * @see Velocity_t
   */
  virtual Velocity_t getVelocity() = 0;

  /**
   * Toggles motor inversion
   * @param inverted whether or not the motor should be inverted. true if inverted.
   */
  virtual void setInverted(bool inverted) = 0;
  
  /**
   * Get whether or not the motor is inverted
   * @return whether or not the motor is inverted. true if inverted.
   */
  virtual bool getInverted() = 0;
};
} // namespace rmb
