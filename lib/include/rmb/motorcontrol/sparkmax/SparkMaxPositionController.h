#pragma once

#include <units/angle.h>
#include <units/base.h>
#include <units/time.h>

#include <rev/CANSparkMax.h>

#include "rmb/motorcontrol/PositionController.h"
#include "rmb/motorcontrol/feedforward/SimpleMotorFeedforward.h" 

namespace rmb {
/**
 * A wrapper around the SparkMax motorcontroller that allows for the user to set and get the position of the motor
 * accurately through PID functionallity
 */
template <typename DistanceUnit>
class SparkMaxPositionController : PositionController<DistanceUnit> {
public:
  using Distance_t = typename PositionController<DistanceUnit>::Distance_t;

  using VeloctyUnit = typename PositionController<DistanceUnit>::VelocityUnit;
  using Velocity_t = typename PositionController<DistanceUnit>::Velocity_t;

  using AccelerationUnit =
      typename PositionController<DistanceUnit>::AccelerationUnit;
  using Acceleration_t =
      typename PositionController<DistanceUnit>::Acceleration_t;

  /**
   * The distance units that the SparkMax takes in by default is rotations. The conversion here
   * is one rotation to 2pi radians.   
   */
  using RawUnit =
      typename units::unit<std::ratio<2>, units::radians, std::ratio<1>>;
  using RawUnit_t = typename units::unit_t<RawUnit>;

  /**
   * The velocity units that the SparkMaxes use are done in rotations/minute.
   */
  using RawVelocity =
      typename units::compound_unit<RawUnit, units::inverse<units::minutes>>;
  using RawVelocity_t = typename units::unit_t<RawVelocity>;
  
  /**
   * The SparkMax takes in acceleration in rpm/second. 
   */
  using RawAccel =
      typename units::compound_unit<RawVelocity,
                                    units::inverse<units::seconds>>;
  using RawAccel_t = typename units::unit_t<RawAccel>;

  /**
   * The conversion unit from the user defined DistanceUnit to radians. See SparkMaxVelocityController<DistanceUnit>::ConversionUnit.
   */
  using ConversionUnit =
      typename units::compound_unit<DistanceUnit,
                                    units::inverse<units::radians>>;
  using ConversionUnit_t = typename units::unit_t<ConversionUnit>;

  /**
   * Configuration constants for the SparkMax's PID controller.
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

  struct Follower {
    int id;
    rev::CANSparkMax::MotorType motorType;
    bool inverted;
  };

  /** \deprecated
   * Creates a SparkMaxPositionController with the specified deviceID
   */
  SparkMaxPositionController(int deviceID);

  /**
   * Creates a SparkMaxPositionController with the specified SparkMax ID, PID configuration, and unit configuration
   * @param deviceID The ID of the target SparkMax motorcontroller
   * @param pidConfig configuration constants for the SparkMax PID controller. These constants will help you tune
   *                  your motor such that it runs smooth within the desired bounds.
   * @param conversion The conversion from the user provided DistanceUnits to radians. 
   *                   See SparkMaxPositionController<DistanceUnit>::ConversionUnit
   */
  SparkMaxPositionController(int deviceID, const PIDConfig &pidConfig,
                             ConversionUnit_t conversion = ConversionUnit_t(1), 
                             const Feedforward<DistanceUnit>& feedForward = noFeedforward<DistanceUnit>,
                             std::initializer_list<Follower> followers = {});

  /**
   * Sets the position of the motor.
   * @param position The distance from the reference, or "zero" point to set the motor to in user provided Distance_t. \n 
   *                 See SparkMaxPositionController<DistanceUnit>::resetReference(Distance_t position)
   */
  void setPosition(Distance_t position) override;

  /**
   * Gets the position of the motor according to the SparkMax encoder
   * @return the distance from the reference point in the user defined Distance. The reference point can be set with 
   *         rmb::SparkMaxPositionController< DistanceUnit >::resetRefrence(Distance_t position) 	
   */
  Distance_t getPosition() override;

  /**
   * Gets the velocity of the motor according to the SparkMax encoder
   */
  Velocity_t getVelocity() override;

  /**
   * Toggles motor inversion based on inverted parameter
   * @param inverted Desired motor inversion status. If this is true, the motor will be inverted.
   */
  inline void setInverted(bool inverted) override {
    sparkMax.SetInverted(inverted);
  };

  /**
   * Gets the inversion status of the motor
   * @return Motor inversion status. If this is true, the motor is inverted.
   */
  inline bool getInverted() override { return sparkMax.GetInverted(); };

  /**
   * Resets the reference point to the provided postiion.
   * @param position The desired reference point. This point will be treated as the 
   *                 "0" for the SparkMaxPositionController<DistanceUnit>::getPosition() 
   *                 and SparkMaxPositionController<DistanceUnit>::setPosition(Distance_t position) calls
   */
  void resetRefrence(Distance_t position) override;

  /**
   * Sets the max allowed position of the motor. If the user attempts to move the motor beyond the maximum position, 
   * the motor will be set to the maximum position.
   * @param max The maximum position in user defined DistanceUnits
   */
  void setMaxPosition(Distance_t max) override;

  /**
   * Get the maximum allowed position of the motor
   * @return The maximum postition in user defined distance units
   */
  Distance_t getMaxPosition() override;

  /**
   * Sets the minimum allowed position of the motor. If the user attempts to move the motor beyond the maximum position, 
   * the motor will be set to the minimum position.
   * @param min The minimum position in user defined distance units
   */
  void setMinPosition(Distance_t min) override;

  /**
   * Get the minimum allowed position of the motor
   * @return The minimum postition in user defined distance units
   */
  Distance_t getMinPosition() override;

private:
  rev::CANSparkMax sparkMax;
  rev::SparkMaxRelativeEncoder sparkMaxEncoder;
  rev::SparkMaxPIDController sparkMaxPIDController;

  ConversionUnit_t conversion;

  RawUnit_t maxPosition = RawUnit_t(1);
  RawUnit_t minPosition = RawUnit_t(-1);

  rev::CANSparkMax::ControlType controlType;
  std::vector<std::unique_ptr<rev::CANSparkMax>> followers;

  const Feedforward<DistanceUnit> &feedforward;
};
} // namespace rmb
