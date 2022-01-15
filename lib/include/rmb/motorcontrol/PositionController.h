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

      virtual void restRefrence(Distance_t position) = 0;
      virtual void setMaxPositon(Distance_t max) = 0;
      virtual Distance_t getMaxPositon() = 0;
      virtual void setMinPosition(Distance_t min) = 0;
      virtual Distance_t getMinPosition() = 0;

      virtual void setPosition(Distance_t position) = 0;
      virtual Distance_t getPosition() = 0;
      virtual Velocity_t getVelocity() = 0;
      virtual Velocity_t getAcceleration() = 0;
  };
} // rmb namespace
