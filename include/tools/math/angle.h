#pragma once

#include <cmath>

#include "types/math_types.h"

namespace ismpc {

/**
 * Converts angle from rad to degrees.
 * @param angle code in rad
 * @return angle coded in degrees
 */
template <typename V>
constexpr V toDegrees(V angle) {
    return angle * V(180.f / M_PI);
}

/**
 * The Angle class stores the represented angle in radiant.
 */
class Angle {
   private:
    Scalar value = 0.f;

   public:
    constexpr Angle() = default;
    constexpr Angle(Scalar angle) : value(angle) {}

    operator Scalar&() {
        return value;
    }
    constexpr operator const Scalar&() const {
        return value;
    }

    constexpr Angle operator-() const {
        return Angle(-value);
    }

    Angle& operator+=(Scalar angle) {
        value += angle;
        return *this;
    }

    Angle& operator-=(Scalar angle) {
        value -= angle;
        return *this;
    }

    Angle& operator*=(Scalar angle) {
        value *= angle;
        return *this;
    }

    Angle& operator/=(Scalar angle) {
        value /= angle;
        return *this;
    }

    Angle& normalize() {
        value = normalize(value);
        return *this;
    }

    /**
     * reduce angle to [-pi..+pi[
     * @param data angle coded in rad
     * @return normalized angle coded in rad
     */
    template <typename V>
    static V normalize(V data);

    Angle diffAbs(Angle b) const {
        return std::abs(normalize(value - b));
    }

    static constexpr Angle fromDegrees(Scalar degrees) {
        return Angle((degrees / 180.f) * M_PI);
    }
    static constexpr Angle fromDegrees(int degrees) {
        return fromDegrees(static_cast<Scalar>(degrees));
    }

    constexpr Scalar toDegrees() const {
        return (value / M_PI) * 180.f;
    }
};

inline constexpr Angle operator"" _deg(unsigned long long int angle) {
    return Angle::fromDegrees(static_cast<Scalar>(angle));
}

inline constexpr Angle operator"" _deg(long double angle) {
    return Angle::fromDegrees(static_cast<Scalar>(angle));
}

inline constexpr Angle operator"" _rad(unsigned long long int angle) {
    return Angle(static_cast<Scalar>(angle));
}

inline constexpr Angle operator"" _rad(long double angle) {
    return Angle(static_cast<Scalar>(angle));
}

template <typename V>
V Angle::normalize(V data) {
    if (data >= -V(M_PI) && data < V(M_PI))
        return data;
    else {
        data = data - static_cast<Scalar>(static_cast<int>(data / V(2.f * M_PI))) * V(2.f * M_PI);
        return data >= V(M_PI) ? V(data - V(2.f * M_PI)) : data < -V(M_PI) ? V(data + V(2.f * M_PI)) : data;
    }
}

}  // namespace ismpc

#ifndef isfinite
namespace std {
inline bool isfinite(ismpc::Angle angle) noexcept {
    return isfinite(static_cast<Scalar>(angle));
}
}  // namespace std
#endif
