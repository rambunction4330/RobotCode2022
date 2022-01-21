// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/base.h>

namespace rmb {
  template<typename Distance>
  class PositionController {
    public:
      using Distance_t = units::unit_t<Distance>;
      using Velocity = units::compound_unit<Distance, units::inverse<units::seconds>>;
      using Velocity_t = units::unit_t<Velocity>;
      using Acceleration = units::compound_unit<Velocity, units::inverse<units::seconds>>;
      using Acceleration_t = units::unit_t<Acceleration>;

      virtual void resetRefrence(Distance_t position) = 0;
      virtual void setMaxPosition(Distance_t max) = 0;
      virtual Distance_t getMaxPosition() const = 0;
      virtual void setMinPosition(Distance_t min) = 0;
      virtual Distance_t getMinPosition() const = 0;

      virtual void setPosition(Distance_t position) = 0;
      virtual Distance_t getPosition() const = 0;
      virtual Velocity_t getVelocity() const = 0;

      virtual void setInverted(bool inverted) = 0;
      virtual bool getInverted() const = 0;
      virtual void disable() = 0;
      virtual void stopMotor() = 0;

  };
} // rmb namespace
