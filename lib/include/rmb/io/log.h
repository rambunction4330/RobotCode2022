#pragma once

#include <string>
#include <wpi/raw_ostream.h>

namespace wpi {
    enum IOFmtOps {
        endl,
        flush
    };
}

wpi::raw_ostream& operator<<(wpi::raw_ostream&os, wpi::IOFmtOps op) {
    switch (op) {
    case wpi::flush:
        os.flush();
        break;
    case wpi::endl:
        os << "\n";
        os.flush();
        break;
    }

    return os;
}

template<typename T>    
wpi::raw_ostream& operator<<(wpi::raw_ostream& os, const T& val) {
    os << std::to_string(val);
    return os;
}