#pragma once

#include <units/angle.h>
#include <units/base.h>
#include <units/length.h>
#include <units/voltage.h>

#include <wpi/MathExtras.h>

#include "rmb/motorcontrol/feedforward/Feedforward.h"

namespace rmb {
template <typename DistanceUnit>
class SimpleMotorFeedforward : public Feedforward<DistanceUnit> {
public:
  using Distance_t = typename Feedforward<DistanceUnit>::Distance_t;
  using VelocityUnit = typename Feedforward<DistanceUnit>::VelocityUnit;
  using Velocity_t = typename Feedforward<DistanceUnit>::Velocity_t;
  using AccelerationUnit = typename Feedforward<DistanceUnit>::AccelerationUnit;
  using Acceleration_t = typename Feedforward<DistanceUnit>::Acceleration_t;

  using KsUnit = typename Feedforward<DistanceUnit>::KsUnit;
  using Ks_t = typename Feedforward<DistanceUnit>::Ks_t;
  using KvUnit = typename Feedforward<DistanceUnit>::KvUnit;
  using Kv_t = typename Feedforward<DistanceUnit>::Kv_t;
  using KaUnit = typename Feedforward<DistanceUnit>::KaUnit;
  using Ka_t = typename Feedforward<DistanceUnit>::Ka_t;

  SimpleMotorFeedforward(Ks_t kS, Kv_t kV, Ka_t kA) : kS(kS), kV(kV), kA(kA) {}

  inline units::volt_t
  calculate(Velocity_t velocity, Distance_t distance = Distance_t(0.0),
            Acceleration_t acceleration = Acceleration_t(0.0)) const override {
    return kS * wpi::sgn(velocity) + kV * velocity + kA * acceleration;
  }

  inline Velocity_t
  maxAchievableVelocity(units::volt_t maxVoltage, Acceleration_t acceleration,
                        Distance_t position = Distance_t(0.0)) const override {
    return (maxVoltage - kS - kA * acceleration) / kV;
  }

  inline Velocity_t
  minAchievableVelocity(units::volt_t maxVoltage, Acceleration_t acceleration,
                        Distance_t position = Distance_t(0.0)) const override {
    return (-maxVoltage + kS - kA * acceleration) / kV;
  }

  inline Acceleration_t maxAchievableAcceleration(
      units::volt_t maxVoltage, Velocity_t velocity,
      Distance_t position = Distance_t(0.0)) const override {
    return (maxVoltage - kS * wpi::sgn(velocity) - kV * velocity) / kA;
  }

  inline Acceleration_t minAchievableAcceleration(
      units::volt_t maxVoltage, Velocity_t velocity,
      Distance_t position = Distance_t(0.0)) const override {
    return maxAchievableAcceleration(-maxVoltage, velocity);
  }

  inline Kv_t getVelocityGain() const override { return kV; }
  inline Ka_t getAcclerationGain() const override { return kA; }

  inline units::volt_t
  calculateStatic(Velocity_t velocity,
                  Distance_t position = Distance_t(0)) const override {
    return kS * wpi::sgn(velocity);
  }

private:
  Ks_t kS;
  Kv_t kV;
  Ka_t kA;
};

template <typename U>
SimpleMotorFeedforward<U>
    noFeedforward(typename SimpleMotorFeedforward<U>::Ks_t(0.0),
                  typename SimpleMotorFeedforward<U>::Kv_t(0.0),
                  typename SimpleMotorFeedforward<U>::Ka_t(0.0));

} // namespace rmb