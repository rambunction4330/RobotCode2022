#pragma once

#include <units/base.h>

namespace rmb {

/**
 * Base class for wrappers around vendor API / device specific positionController code. 
 * This class provides a standard interface to code that allows you to control the position of a motor.
 */
template <typename DistanceUnit> class PositionController {
public:
  using Distance_t = units::unit_t<DistanceUnit>; /**< User specified distance unit. Is probably going to be an angle or a length*/
  using VelocityUnit =
      units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>; /**< DistanceUnit per second*/
  using AccelerationUnit =
      units::compound_unit<VelocityUnit, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<AccelerationUnit>; /**< DistanceUnit per second squared */

  /**
   * Resets the reference point of the position controller to base position readings off of.
   * @param position the desired reference point of the position controller in user specified DistanceUnits
   */
  virtual void resetRefrence(Distance_t position) = 0;

  /**
   * Set the maximum position that the motor controller can go to.
   * @param max the maximum position in user specified DistanceUnits
   */
  virtual void setMaxPosition(Distance_t max) = 0;

  /**
   * Get the maximum position of the motorcontroller
   * @return the maximum position in user specified DistanceUnits
   */
  virtual Distance_t getMaxPosition() = 0;

  /**
   * Set the minimum position the motorcontroller can go to.
   * @param min the minimum position in user specified DistanceUnits
   */
  virtual void setMinPosition(Distance_t min) = 0;

  /**
   * Get the minimum position the motorcontroller can go to
   * @return the minimum position in user specified DistanceUnits
   */
  virtual Distance_t getMinPosition() = 0;


  /**
   * Sets the desired position of the motor.
   * @param position the desired position in user specified DistanceUnits
   */
  virtual void setPosition(Distance_t position) = 0;


  /**
   * Get the current position of the motor.
   * @return the current position of the motor in user specified DistanceUnits
   */
  virtual Distance_t getPosition() const = 0;

  /**
   * Get the current velocity of the motor
   * @return the current velocity of the motor in user specified DistanceUnits
   */
  virtual Velocity_t getVelocity() = 0;

  /**
   * Invert the motor (make it go reverse from the default)
   * @param inverted whether or not the motor should be inverted. true if inverted.
   */
  virtual void setInverted(bool inverted) = 0;

  /**
   * Get whether or not the motor is inverted
   * @return true if the motor is inverted
   */
  virtual bool getInverted() = 0;
};

} // namespace rmb
