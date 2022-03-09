//@notSam25
#pragma once

#include <cmath>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/JoystickButton.h>

#include <rmb/math/misc.h>
#include <rmb/io/log.h>

class JoystickSubsystem : public frc2::SubsystemBase {
public:
  JoystickSubsystem(int port = 0, float dz = 0.1f, bool sqrTwst = false)
      : joystick(port), deadZone(dz), squareTwist(sqrTwst) {
  } // Init constructer

  double getX() const {
      double val = std::abs(joystick.GetX()) < deadZone ? 0.0 : joystick.GetX();
      return -1.0 * wpi::sgn(val) * rmb::map(std::abs(val),
                                             deadZone, 0.0,
                                             1.0, 1.0, false);

  }

  double getY() const {
      double val = std::abs(joystick.GetY()) < deadZone ? 0.0 : joystick.GetY();
      return -1.0 * wpi::sgn(val) * rmb::map(std::abs(val),
                                             deadZone, 0.0,
                                             1.0, 1.0, false);
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

    double twist = std::abs(joystick.GetTwist()) < deadZone ? 0.0 : joystick.GetTwist();

    wpi::outs() << "raw twist: " << twist;
    if(squareTwist) {
        // -1 * 0.09 -> -0.09
        //twist = wpi::sgn(twist) * std::pow(twist, 2);
    }

    // -1 *
    twist = wpi::sgn(twist) * rmb::map(std::abs(twist), deadZone, 0.0, 1.0, 1.0, false);


    wpi::outs() << " twist: " << twist << wpi::endl;
    return twist;
  }

  double getThrottle() const { return -(joystick.GetThrottle() - 1) / 2; }

private:
  frc::Joystick joystick;
  float deadZone = 0.1f;
  bool squareTwist = false;
};
