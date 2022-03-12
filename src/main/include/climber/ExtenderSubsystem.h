#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

class ExtenderSubsystem : public frc2::SubsystemBase {
 public:
  ExtenderSubsystem();

  void extendArm();
  void retractArm();
  void stopArm();
  void Periodic() override;

 private:
  using TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;
  TalonSRX talon{ 51 };
};