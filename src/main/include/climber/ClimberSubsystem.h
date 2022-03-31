#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void extendArm();
  void retractArm();
  void stopArm();
  void Periodic() override;

 private:
  using TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;
  TalonSRX leftTalon{ 51 };
  TalonSRX rightTalon{ 52 };
};
