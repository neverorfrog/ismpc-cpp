#pragma once

#include <stdexcept>
#include <string>

namespace ismpc {

enum class Arm : int { left, right, numOfArms };

enum class Leg : int { left, right, numOfLegs };

enum class Foot : int { left, right, numOfFeet };

inline std::string toString(Foot foot) {
    switch (foot) {
        case Foot::left:
            return "LEFT";
        case Foot::right:
            return "RIGHT";
        default:
            return "UNKNOWN";
    }
}

inline std::ostream& operator<<(std::ostream& os, Foot foot) {
    os << toString(foot);
    return os;
}

}  // namespace ismpc
