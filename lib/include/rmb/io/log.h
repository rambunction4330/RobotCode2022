#pragma once

#include <string>
#include <wpi/raw_ostream.h>
#include <units/base.h>

namespace wpi {

    template<typename T>
    raw_ostream& operator<<(raw_ostream& os, units::unit_t<T> val) {
        os << val();
        return os;
    }

    /**
     * Fallback in case none of the `raw_ostream& operator<<` functions match your function call. This will help
     * you print out doubles, floats, ints and any other type that std::to_string supports 
     */
    template<typename T>    
    raw_ostream& operator<<(raw_ostream& os, const T& val) {
        os << std::to_string(val);
        return os;
    }

    /**
     * Lets you pass functions to `raw_ostream& operator<<`. This then allows you to do what
     * the c++ standard library does with std::cout << std::endl where std::endl is a function
     * that takes in an ostream reference and returns that ostream reference so that you can chain
     * commands.
     * @see raw_ostream& flush(raw_ostream& os)
     * @see raw_ostream& endl(raw_ostream& os)
     */
    inline raw_ostream& operator<<(raw_ostream& os, raw_ostream& (*pf)(raw_ostream&)) {
        return (*pf)(os);
    }
    
    /**
     * Flushes the output so that you can see it on the RioLog before the output buffer fills up. Usage:
     * 
     *     wpi::outs() << "text" << wpi::flush;
     * 
     */
    inline raw_ostream& flush(raw_ostream& os) {
        os.flush();
        return os;
    }

    /**
     * Prints a newline and flushes the output. The equivelent of
     * 
     *     wpi::outs() << "\n" << wpi::flush;
     * 
     * @see raw_ostream& endl(raw_ostream& os)
     */
    inline raw_ostream& endl(raw_ostream& os) {
        os << "\n" << flush;
        return os;
    }
}
