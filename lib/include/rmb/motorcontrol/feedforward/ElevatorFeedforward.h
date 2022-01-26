#pragma once

#include <units/base.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/voltage.h>

#include <frc/controller/ElevatorFeedforward.h>

#include "rmb/motorcontrol/feedforward/Feedforward.h"

namespace rmb {

template <typename DistanceUnit>
class ElevatorFeedforward : public Feedforward<DistanceUnit>{
public:

  using Distance_t = typename units::unit_t<DistanceUnit>;
  using VelocityUnit = typename Feedforward<DistanceUnit>::VelocityUnit;
  using Velocity_t = typename Feedforward<DistanceUnit>::Velocity_t;
  using AccelerationUnit = typename Feedforward<DistanceUnit>::AccelerationUnit;
  using Acceleration_t = typename Feedforward<DistanceUnit>::Acceleration_t;

  using KvUnits = typename frc::ElevatorFeedforward<DistanceUnit>::kv_unit;
  using Kv_t = typename units::unit_t<KvUnits>;
  using KaUnits = typename frc::ElevatorFeedforward<DistanceUnit>::ka_unit;
  using Ka_t = typename units::unit_t<KaUnits>;

  ElevatorFeedforward(units::volt_t kS, units::volt_t kG, Kv_t kV, Ka_t kA) : feedforward{kS, kG, kV, kA} {}

  inline units::volt_t
  calculate(Velocity_t velocity, Distance_t distance = Distance_t(0.0),
            Acceleration_t acceleration = Acceleration_t(0.0)) override {
    return feedforward.Calculate(velocity, acceleration);
  }

  Velocity_t maxAchievableVelocity(units::volt_t maxVoltage, Acceleration_t acceleration, Distance_t position = Distance_t(0.0)) override {
    return feedforward.MaxAchievableVelocity(maxVoltage, acceleration); 
  }
  Velocity_t minAchievableVelocity(units::volt_t maxVoltage, Acceleration_t acceleration, Distance_t position = Distance_t(0.0)) override {
    return feedforward.MinAchievableVelocity(maxVoltage, acceleration);
  }
  Acceleration_t maxAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity, Distance_t position = Distance_t(0.0)) override {
    return feedforward.MaxAchievableAcceleration(maxVoltage, velocity);
  }
  Acceleration_t minAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity, Distance_t position = Distance_t(0.0)) override {
    return feedforward.MinAchievableAcceleration(maxVoltage, velocity);
  }  

private:
  frc::ElevatorFeedforward<DistanceUnit> feedforward;
  
};
}