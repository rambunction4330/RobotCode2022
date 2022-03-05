#include <rmb/io/log.h>
#include "rmb/motorcontrol/sparkmax/SparkMaxError.h"

// false if error. True if not.
bool checkREVLibError(rev::REVLibError error, const char *functionCall,
                      const char *fileName, int line) {
  // map rev::REVLibError enum to const char*
  static const char *REVLibErrorStrings[]{
      "kOk",
      "kError",
      "kTimeout",
      "kNotImplemented",
      "kHALError",
      "kCantFindFirmware",
      "kFirmwareTooOld",
      "kFirmwareTooNew",
      "kParamInvalidID",
      "kParamMismatchType",
      "kParamAccessMode",
      "kParamInvalid",
      "kParamNotImplementedDeprecated",
      "kFollowConfigMismatch",
      "kInvalid",
      "kSetpointOutOfRange",
      "kUnknown",
      "kCANDisconnected",
      "kDuplicateCANId",
      "kInvalidCANId",
      "kSparkMaxDataPortAlreadyConfiguredDifferently"};

  if (error == rev::REVLibError::kOk) {
    // do nothing. Should be optimized out
    return true;
  }
  wpi::outs() << "[ SparkMax ERROR " << REVLibErrorStrings[(int)error]
              << " ] in " << fileName << ":" << std::to_string(line) << " \""
              << functionCall << "\"" << wpi::endl;


  return false;
}
