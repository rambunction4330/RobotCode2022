#pragma once

#include <units/angle.h>
#include <units/base.h>

#include "rmb/motorcontrol/VelocityController.h"
#include <rev/CANSparkMax.h>

#include <frc/MotorSafety.h>

namespace rmb {

/**
 * A wrapper around the SparkMax motorcontroller that allows for the user to set and get the velocity of the motor
 * accurately through PID functionallity
 */
template <typename DistanceUnit>
class SparkMaxVelocityController : public VelocityController<DistanceUnit> {
public:
  using Distance_t = typename VelocityController<DistanceUnit>::Distance_t;

  using VeloctyUnit = typename VelocityController<DistanceUnit>::VelocityUnit;
  using Velocity_t = typename VelocityController<DistanceUnit>::Velocity_t;

  using AccelerationUnit =
      typename VelocityController<DistanceUnit>::AccelerationUnit;
  using Acceleration_t =
      typename VelocityController<DistanceUnit>::Acceleration_t;

  using ConversionUnit =
      typename units::compound_unit<DistanceUnit,
                                    units::inverse<units::radians>>; /**< The ratio between the user-defined DistanceUnit and radians. 
                                                                          This is used for converting to and from user-defined units and
                                                                          raw units. For example, if a wheel has a wheel with a circumference
                                                                          of 2 meters, the conversion from meters to radians (2pi = 1 rotation) 
                                                                          would be:

                                                                              2 meters / 2pi radians
                                                                              (1/pi) m/rad
                                                                              conversion ≈ 0.3183
                                                                          
                                                                          */
  using ConversionUnit_t = typename units::unit_t<ConversionUnit>;

  // raw velocity is in rpm. 1 rotation * 2pi rad = 2pi rad
  using RawUnit =
      typename units::unit<std::ratio<2>, units::radians, std::ratio<1>>; /**< The SparkMax does it's distances in rotations. 
                                                                               The rotations quantity is based on radians. The math for the conversion
                                                                               is as follows:

                                                                                   1 rotation * 2pi rad = 2pi rad
                                                                               
                                                                               Thus the multiplier provided to the units:: library is 2pi
                                                                          */
  using RawUnit_t = typename units::unit_t<RawUnit>;
  using RawVelocity =
      typename units::compound_unit<RawUnit, units::inverse<units::minutes>>; /**< The SparkMax does it's velocities in rotations per minute */
  using RawVelocity_t = typename units::unit_t<RawVelocity>;
  using RawAccel =
      typename units::compound_unit<RawVelocity,
                                    units::inverse<units::seconds>>; /**< Oddly, the SparkMax performs it's acceleration calculations in rotations per 
                                                                          minute per second. */
  using RawAccel_t = typename units::unit_t<RawAccel>;
  
  /**
   * The PID constants used by the constructor of SparkMaxVelocityController. 
   */
  struct PIDConfig {
    double p, i, d, f;
    double iZone, iMaxAccumulator;
    double maxOutput, minOutput;

    // SmartMotion config
    bool usingSmartMotion;
    Velocity_t maxVelocity, minVelocity;
    Acceleration_t maxAccel;
    Distance_t allowedErr;
    rev::SparkMaxPIDController::AccelStrategy accelStrategy;
  };

  /** \deprecated
   * Creates a new SparkMaxVelocityController with the specified device id
   * @param deviceID the ID of the target SparkMax motor controller
   */
  SparkMaxVelocityController(int deviceID);

  /**
   * Creates a SparkMaxVelocityController
   * @param deviceID the ID of the target SparkMax motor controller
   * @param config PID constants to be set on the SparkMax's PID controller. 
   *               These are set once.
   * @param conversionUnit This is the ratio between DistanceUnits(the units that the user passes in)
   *                       and units::radians. For example, if the unit passed in is rotations, the process
   *                       to find the conversion unit would be \n 
   *                      
   *                           1 rotation / 2pi rad =>
   *                           1/2pi rot/rad 
   *                           conversion ≈ 0.1592
   *                       
   */
  SparkMaxVelocityController(int deviceID, const PIDConfig &config,
                             ConversionUnit_t conversionUnit = 1);

  /**
   * Sets the target velocity of the motorcontroller
   * @param velocity The target velocity
   */
  void setVelocity(Velocity_t velocity) override;

  /**
   * Returns the current velocity of the motor
   * @return The current velocity of the motor
   */
  Velocity_t getVelocity() override;

  /**
   * Toggles motorcontroller inversion
   * @param inverted If this is true, the motorcontroller will be inverted
   */
  inline void setInverted(bool inverted) override {
    sparkMax.SetInverted(inverted);
  };

  /**
   * Get the inversion status of the motorcontroller
   * @return The inversion status of the motorcontroller. 
   *          If this is true, the motor controller is inverted
   */
  inline bool getInverted() override { return sparkMax.GetInverted(); };

private:
  rev::CANSparkMax sparkMax;
  rev::SparkMaxRelativeEncoder sparkMaxEncoder;
  rev::SparkMaxPIDController sparkMaxPIDController;
  ConversionUnit_t conversion;

   rev::CANSparkMax::ControlType controlType;
};
} // namespace rmb
