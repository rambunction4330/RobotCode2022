#pragma once

#include <units/angle.h>
#include <units/base.h>
#include <units/voltage.h>

#include <frc/controller/ArmFeedforward.h>

#include <rmb/motorcontrol/feedforward/Feedforward.h>

namespace rmb {

class ArmFeedforward : public Feedforward<units::radians> {
public:
  using Distance_t = typename Feedforward<units::radians>::Distance_t;
  using VelocityUnit = typename Feedforward<units::radians>::VelocityUnit;
  using Velocity_t = typename Feedforward<units::radians>::Velocity_t;
  using AccelerationUnit =
      typename Feedforward<units::radians>::AccelerationUnit;
  using Acceleration_t = typename Feedforward<units::radians>::Acceleration_t;

  using KsUnit = typename Feedforward<units::radians>::KsUnit;
  using Ks_t = typename Feedforward<units::radians>::Ks_t;
  using KvUnit = typename Feedforward<units::radians>::KvUnit;
  using Kv_t = typename Feedforward<units::radians>::Kv_t;
  using KaUnit = typename Feedforward<units::radians>::KaUnit;
  using Ka_t = typename Feedforward<units::radians>::Ka_t;

  ArmFeedforward(Ks_t kS, Ks_t kCos, Kv_t kV, Ka_t kA)
      : kS(kS), kCos(kCos), kV(kV), kA(kA){};

  inline units::volt_t calculate(Velocity_t velocity, Distance_t position,
                                 Acceleration_t acceleration) const override {
    return kS * wpi::sgn(velocity) + kCos * units::math::cos(position) +
           kV * velocity + kA * acceleration;
  }

  inline Velocity_t maxAchievableVelocity(units::volt_t maxVoltage,
                                          Acceleration_t acceleration,
                                          Distance_t position) const override {
    return (maxVoltage - kS - kCos * units::math::cos(position) -
            kA * acceleration) /
           kV;
  }

  inline Velocity_t minAchievableVelocity(units::volt_t maxVoltage,
                                          Acceleration_t acceleration,
                                          Distance_t position) const override {
    return (-maxVoltage + kS - kCos * units::math::cos(position) -
            kA * acceleration) /
           kV;
  }

  inline Acceleration_t
  maxAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity,
                            Distance_t position) const override {
    return (maxVoltage - kS * wpi::sgn(velocity) -
            kCos * units::math::cos(position) - kV * velocity) /
           kA;
  }

  inline Acceleration_t
  minAchievableAcceleration(units::volt_t maxVoltage, Velocity_t velocity,
                            Distance_t position) const override {
    return maxAchievableAcceleration(-maxVoltage, velocity, position);
  }

  inline Kv_t getVelocityGain() const override { return kV; }
  inline Ka_t getAcclerationGain() const override { return kA; }

  inline units::volt_t
  calculateStatic(Velocity_t velocity,
                  Distance_t position = Distance_t(0)) const override {
    return kS * wpi::sgn(velocity) + kCos * units::math::cos(position);
  }

private:
  Ks_t kS, kCos;
  Kv_t kV;
  Ka_t kA;
};

} // namespace rmb
