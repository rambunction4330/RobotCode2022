#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void climb();
  void lower();
  void stop();
  void Periodic() override;

 private:
  using TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;
  TalonSRX talon{ 52 };
};
