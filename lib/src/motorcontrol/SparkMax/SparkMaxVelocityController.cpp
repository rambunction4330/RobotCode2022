#include <rmb/motorcontrol/SparkMax/SparkMaxError.h>
#include <rmb/motorcontrol/SparkMax/SparkMaxVelocityController.h>

namespace rmb {

  template <typename U>
  SparkMaxVelocityController<U>::SparkMaxVelocityController(int deviceID) : 
                                                            sparkMax(deviceID, rev::CANSparkMax::MotorType::kBrushless) {
    sparkMaxEncoder = sparkMax.GetEncoder();
    sparkMaxPIDController = sparkMax.GetPIDController();

  }

  template <typename U>
  SparkMaxVelocityController<U>::SparkMaxVelocityController(
                                                            int deviceID, 
                                                            const PIDConfig& config, 
                                                            ConversionUnit_t conversionUnit
                                                            ) : 
                                                            sparkMax(
                                                                    deviceID, 
                                                                    rev::CANSparkMax::MotorType::kBrushless
                                                                    ), conversion(conversionUnit) {
    
    sparkMax.RestoreFactoryDefaults();

    sparkMaxEncoder = sparkMax.GetEncoder();
    sparkMaxPIDController = sparkMax.GetPIDController();

    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetP(config.p));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetI(config.i));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetD(config.d));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetFF(config.f));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetIZone(config.iZone));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetIMaxAccum(config.iMaxAccumulator));
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetOutputRange(config.minOutput, config.maxOutput));
    
    if(config.usingSmartMotion) {
      CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionAllowedClosedLoopError(
        RawUnit_t(config.allowedErr / conversion).to<double>()
      ));
      CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionMaxVelocity(
        RawVelocity_t(config.maxVelocity / conversion).to<double>()
      ));
      CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionMaxAccel(
        RawAccel_t(config.maxAccel / conversion).to<double>()
      ));
      CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionAccelStrategy(config.accelStrategy));
      CHECK_REVLIB_ERROR(sparkMaxPIDController.SetSmartMotionMinOutputVelocity(
        RawVelocity_t(config.minVelocity / conversion).to<double>()
      ));
    }
}

  template <typename U>
  void SparkMaxVelocityController<U>::setVelocity(Velocity_t velocity) {
    double setPoint = RawVelocity_t(velocity / conversion).to<double>;
    CHECK_REVLIB_ERROR(sparkMaxPIDController.SetReference(setPoint, rev::ControlType::kVelocity));
  }

  template <typename U>
  typename SparkMaxVelocityController<U>::Velocity_t SparkMaxVelocityController<U>::getVelocity() {
    return Velocity_t(RawVelocity_t(sparkMaxEncoder.GetVelocity()) * conversion);
  }

} // namespace rmb