#include "driverstation/JoystickSubsystem.h"

double JoystickSubsystem::getX() const {  
    double val = std::abs(joystick.GetX()) < deadZone ? 0.0 : joystick.GetX();
    return coefficients.x * wpi::sgn(val) * rmb::map(std::abs(val),
                                            deadZone, 1.0,
                                            0.0, 1.0, false);
}

double JoystickSubsystem::getY() const {
      double val = std::abs(joystick.GetY()) < deadZone ? 0.0 : joystick.GetY();
      return coefficients.y * wpi::sgn(val) * rmb::map(std::abs(val),
                                             deadZone, 1.0,
                                             0.0, 1.0, false);
}

double JoystickSubsystem::getTwist() const {

    double twist = std::abs(joystick.GetTwist()) < deadZone ? 0.0 : joystick.GetTwist();
   
    twist = coefficients.twist * wpi::sgn(twist) * rmb::map(std::abs(twist), deadZone, 1.0, 0.0, 1.0, false);

    //wpi::outs() << "Twist" << joystick.GetTwist() << " -> " << twist << wpi::endl;

    return twist;
}

double JoystickSubsystem::getThrottle() const { 
    return -(joystick.GetThrottle() - 1) / 2; 
}

frc2::JoystickButton
JoystickSubsystem::getButton(int button) {
    return frc2::JoystickButton(&joystick, button);
}

bool JoystickSubsystem::buttonPressed(int button) {
    return joystick.GetRawButton(button);
}