#pragma once

#include <rev/REVLibError.h>
#include <string>
#include <wpi/raw_ostream.h>

// false if error. True if not.
bool checkREVLibError(rev::REVLibError error, const char *functionCall,
                      const char *fileName, int line);

#define CHECK_REVLIB_ERROR(x) checkREVLibError(x, #x, __FILE__, __LINE__)