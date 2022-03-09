//@notSam25. Revised by @theVerySharpFlat
#pragma once

#include <cmath>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/JoystickButton.h>

#include <rmb/math/misc.h>
#include <rmb/io/log.h>

class JoystickSubsystem : public frc2::SubsystemBase {
public:

  /**
   * A struct that contains the multiplers for the joystick coefficients
   */
  struct JoystickCoefficients {
    double x = 1.0; /**< The multiplier on the x value*/
    double y = 1.0; /**< The multiplier on the y value*/
    double twist = 1.0; /**< The multiplier on the twist value*/
  };

  JoystickSubsystem(int port = 0, double dz = 0.1, JoystickCoefficients joystickCoefficients = {1.0, 1.0, 1.0})
      : joystick(port), deadZone(dz), coefficients(joystickCoefficients) {}

  /**
   * Get the X value of the joystick
   * @return the displacement about the x axis of the joystick between -1.0 and 1.0/ 
   *         If the value is within the deadzone, it is set to zero.
   *         If nonzero the output of this function is then mapped from 
   *         the range [deadzone, 1.0] to [0.0, 1.0].
   *         The rationale for this mapping comes from the fact that when not
   *         mapped, the moment the joystick exits the deadzone, it would return
   *         the raw position of the joystick and the motor connected to the joystick
   *         would jump from zero to moving at a much faster speed. When it is mapped,
   *         the joystick zeroes at the deadzone and increases linearly to 1.0
   */
  double getX() const;

  /**
   * Get the Y value of the joystick
   * @return the displacement about the y axis of the joystick between -1.0 and 1.0
   *         If the value is within the deadzone, it is set to zero.
   * @see getX() getTwist()
   */
  double getY() const;

  /**
   * Get the twist value of the joystick between -1.0 and 1.0
   * @return the displacement about the twist axis of the joystick
   *         If the value is within the deadzone, it is set to zero.
   * @see getX() getY()
   */
  double getTwist() const;

  /**
   * Get the throttle value between 0.0 and 1.0
   * @return the throttle value on the joystick
   */
  double getThrottle() const;

  /**
   * Get the raw JoysticButton from the button code
   * @param button The target button code
   * @return the raw frc::JoystickButton with ID "button"
   */
  frc2::JoystickButton getButton(int button);
  
  /**
   * Check if a button is pressed
   * @param button the ID of the target button on the joystick
   * @return whether or not the button is pressed. true if pressed.
   */
  bool buttonPressed(int button);


  /**
   * Get the raw x value directly from the joystick without parsing
   * @return The raw x value without deadzones
   */
  inline double getXRaw() const {
    return joystick.GetX();
  }


  /**
   * Get the raw y value directly from the joystick without parsing
   * @return The raw y value without deadzones
   */
  inline double getYRaw() const {
    return joystick.GetY();
  }

  /**
   * Get the raw twist value directly from the joystick without parsing
   * @return The raw twist value without deadzones
   */
  inline double getTwistRaw() const {
    return joystick.GetTwist();
  }

  /**
   * Get the raw throttle value directly from the joystick without parsing
   * @return The raw throttle value without deadzones
   */
  inline double getThrottleRaw() const {
    return joystick.GetThrottle();
  }

private:
  frc::Joystick joystick;
  double deadZone = 0.1f;

  JoystickCoefficients coefficients;
};
