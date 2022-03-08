#pragma once

#include <cmath>

namespace rmb {

    inline double constrain(double n, double low, double high) {
        return fmax(fmin(n, high), low);
    }

    inline double map(double n, double start1, double start2, double stop1, double stop2, bool withinBounds = true) {
        const double newval = (n - start1) / (stop1 - start1) * (stop2 - start2) + start2;
        if (!withinBounds) {
            return newval;
        }
        if (start2 < stop2) {
            return constrain(newval, start2, stop2);
        } else {
            return constrain(newval, stop2, start2);
        }
    }
}
