#pragma once

#include <string>
#include <wpi/raw_ostream.h>

namespace rmb {

class OutStream {
public:
    OutStream(){}

    const OutStream& operator<<(const char* str) const {
        wpi::outs() << str;
        return *this;
    }

    const OutStream& operator<<(const std::string& str) const {
        wpi::outs() << str;
        return *this;
    }

    template<typename T>
    const OutStream& operator<<(const T& val) const {
        wpi::outs() << std::to_string(val);
        return *this;
    }
};

const OutStream& outs();

} // namespace rmb