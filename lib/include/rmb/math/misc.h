#pragma once

#include <algorithm>
#include <cmath>

namespace rmb {

    /**
     * Checks if a value is within raneg
     * @param n the value to check for position in range
     * @param lo the lower bound of the range
     * @param hi the upper bound of the range
     * @return whether or not the value is in between lo and hi. true if in between.
     */
    template <typename T>
    inline bool withinRange(T n, T lo, T hi) {
        if (hi < lo) 
            std::swap(lo, hi);
        
        return n > lo && n < hi;
    }

    /**
     * Map a number of one range of numbers to another range of numbers
     * @param n the value to be mapped
     * @param start1 the lower bound of the current range of n
     * @param stop1  the upper bound of the current range of n
     * @param start2 the lower bound of the destination range of n
     * @param stop2  the upper bound of the destination range of n
     * @param withinBounds 
     * @return n mapped to range [start2, stop2]
     */
    template <typename T>
    inline T map(T n, T start1,  T stop1, T start2, T stop2, bool withinBounds = true) {
        const T newval = (n - start1) / (stop1 - start1) * (stop2 - start2) + start2;
        if (!withinBounds) {
            return newval;
        }
        if (start2 < stop2) {
            return std::clamp(newval, start2, stop2);
        } else {
            return std::clamp(newval, stop2, start2);
        }
    }
}
