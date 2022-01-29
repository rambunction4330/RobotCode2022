// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/acceleration.h>
#include <units/length.h>
#include <units/velocity.h>

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <frc/geometry/Translation2d.h>

#include <AHRS.h>

#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>
#include <rmb/motorcontrol/feedforward/SimpleMotorFeedforward.h>

namespace driveSubsystemConstants {
const static frc::SPI::Port gyroPort = frc::SPI::kMXP;

const static int frontLeftID = 1, frontRightID = 2, rearLeftID = 3,
                 rearRightID = 4;

const static frc::Translation2d frontLeftPose(-0.381_m, 0.381_m),
    frontRightPose(0.381_m, 0.381_m), rearLeftPose(-0.381_m, -0.381_m),
    rearRightPose(0.381_m, -0.381_m);

const static rmb::SparkMaxVelocityController<units::meters>::PIDConfig
    motorPIDConfig{
        /* p */ 0.0057181, /* i */ 0.0, /* d */ 0.0, /* f */ 0.0,
        /* iZone */ 0.0, /* iMaxAccumulator */ 0.0,
        /* maxOutput */ 1.0, /* minOutput */ -1.0,

        /* SmartMotion config */
        /* usingSmartMotion */ true,
        /* maxVelocity */ 25_mps, /* minVelocity */ 0_mps,
        /* maxAccel */ 10_mps_sq,
        /* allowedErr */ 0.9_mps,
        /* accelStrategy */ rev::SparkMaxPIDController::AccelStrategy::kSCurve};

static rmb::SimpleMotorFeedforward<units::meters>
    motorFeedforward(rmb::SimpleMotorFeedforward<units::meters>::Ks_t(0.0),
                     rmb::SimpleMotorFeedforward<units::meters>::Kv_t(0.0),
                     rmb::SimpleMotorFeedforward<units::meters>::Ka_t(0.0));

const static rmb::SparkMaxVelocityController<units::meters>::ConversionUnit_t
    motorConvertion(0.04);

const static units::meters_per_second_t maxVelocity(0.0);
const static units::radians_per_second_t maxRotVelocity(0.0);
} // namespace driveSubsystemConstants