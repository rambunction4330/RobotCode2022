#pragma once

#include <string>
#include <wpi/raw_ostream.h>

template<typename T>
wpi::raw_ostream& operator<<(wpi::raw_ostream& os, const T& val) {
    os << std::to_string(val);
    return os;
} 