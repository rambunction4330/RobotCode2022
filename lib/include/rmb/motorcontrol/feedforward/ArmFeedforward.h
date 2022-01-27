#pragma once

#include <units/angle.h>

#include <frc/controller/ArmFeedforward.h>

#include <rmb/motorcontrol/feedforward/Feedforward.h>

namespace rmb {

class ArmFeedforward : public Feedforward<units::radians> {
public:
  using Distance_t = typename units::unit_t<units::radians>;
  using VelocityUnit = typename Feedforward<units::radians>::VelocityUnit;
  using Velocity_t = typename Feedforward<units::radians>::Velocity_t;
  using AccelerationUnit = typename Feedforward<units::radians>::AccelerationUnit;
  using Acceleration_t = typename Feedforward<units::radians>::Acceleration_t;

  using KaUnit = typename frc::ArmFeedforward::ka_unit;
  using Ka_t = typename units::unit_t<KaUnit>;
  using KvUnit = typename frc::ArmFeedforward::kv_unit;
  using Kv_t = typename units::unit_t<KvUnit>;

  ArmFeedforward(units::volt_t kS, units::volt_t kCos, Kv_t kV, Ka_t ka) : feedforward{kS, kCos, kV, ka} {};

  units::volt_t calculate(Velocity_t velocity, Distance_t angle,
            Acceleration_t acceleration) override{
    return feedforward.Calculate(angle, velocity, acceleration);
  }

  Velocity_t maxAchievableVelocity(units::volt_t maxVoltage, Acceleration_t acceleration, Distance_t position) override {
    return feedforward.MaxAchievableVelocity(maxVoltage, position, acceleration);
  }

  Velocity_t minAchievableVelocity(units::volt_t maxVoltage, Acceleration_t acceleration, Distance_t position) override {
    return feedforward.MinAchievableVelocity(maxVoltage, position, acceleration);
  }

  Acceleration_t maxAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity, Distance_t position) override {
    return feedforward.MaxAchievableAcceleration(maxVoltage, position, velocity);
  }

  Acceleration_t minAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity, Distance_t position) override {
    return feedforward.MinAchievableAcceleration(maxVoltage, position, velocity);
  } 

private:
  frc::ArmFeedforward feedforward;

};

}
