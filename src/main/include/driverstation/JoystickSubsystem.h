//@notSam25
#pragma once

#include <cmath>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/JoystickButton.h>

class JoystickSubsystem : public frc2::SubsystemBase {
public:
  JoystickSubsystem(int port = 0, float dz = 0.1f, bool sqrTwst = false)
      : joystick(port), deadZone(dz), squareTwist(sqrTwst) {
  } // Init constructer

  double getX() const {
    return std::abs(joystick.GetX()) <= deadZone
               ? 0.0f
               : -joystick.GetX(); // Get the Y val of the joystick, doesn't
                                  // include values within deadzone
  }

  double getY() const {
    return std::abs(joystick.GetY()) <= deadZone
               ? 0.0f
               : -joystick.GetY(); // Get the Y val of the joystick, doesn't
                                  // include values within deadzone
  }

  inline double getXRaw() const { // Gets Joystick Raw values witout deadzone
    return joystick.GetX();
  }

  inline double getYRaw() const { // Gets Joystick Raw values witout deadzone
    return joystick.GetY();
  }

  inline double getTwistRaw() const {
    return joystick.GetTwist();
  }
  frc2::JoystickButton
  getButton(int button) { // Gets the button passed through the enum
    return frc2::JoystickButton(&joystick, button);
  }
  
  bool
  buttonPressed(int button) { // Gets the button passed through the enum
    return joystick.GetRawButton(button);
  }

  double getTwist() const { // Gets joystick twist/rotation
    double twist = joystick.GetTwist();
    if (std::abs(twist) <= deadZone) {
      twist = 0.0;
    } else {
      if (squareTwist) {
        twist = std::pow(twist, 2);
        twist = (joystick.GetTwist() >= 0) ? twist : -twist;
      }
    } // If none, then it just returns twist
    return twist;
  }

  double getThrottle() const { return -(joystick.GetThrottle() - 1) / 2; }

private:
  frc::Joystick joystick;
  float deadZone = 0.1f;
  bool squareTwist = false;
};
