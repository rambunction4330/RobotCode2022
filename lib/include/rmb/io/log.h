#pragma once

#include <string>
#include <wpi/raw_ostream.h>

namespace rmb {

/** Wrapper around the wpi::raw_ostring class
 * This class allows you to print things other const char*s and std::strings using the functionallity of std::to_string.
 * This is intended to make your life easier, So use it! 
 */
class OutStream {
public:
    /**
     * creates an OutStream. You can (and probably should) use rmb::outs() to get an rmb::OutStream
     */
    OutStream(){}

    /**
     * Logs a c-style string
     * @param str the cstring to be logged
     */
    const OutStream& operator<<(const char* str) const {
        wpi::outs() << str;
        return *this;
    }

    /**
     * Logs an std::string
     * @param str the std::string to be logged
     */
    const OutStream& operator<<(const std::string& str) const {
        wpi::outs() << str;
        return *this;
    }

    /**
     * The fallback overload for the operator<< function. Used if the type is not a const char* or const std::string&
     * @param val the value to be logged. Can be anything that std::to_string can handle
     */
    template<typename T>
    const OutStream& operator<<(const T& val) const {
        wpi::outs() << std::to_string(val);
        return *this;
    }

};

/**
 * Returns an rmb::OutStream. This is guaranteed to be the same rmb::OutStream every time the function is called.
 */
const OutStream& outs();

} // namespace rmb