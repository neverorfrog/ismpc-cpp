#pragma once
#include <string>

namespace ismpc {

enum class WalkPhase { STARTING, WALKING, STOPPING, FALLING };

inline std::string toString(WalkPhase walkPhase) {
    switch (walkPhase) {
        case WalkPhase::STARTING:
            return "STARTING";
        case WalkPhase::WALKING:
            return "WALKING";
        case WalkPhase::STOPPING:
            return "STOPPING";
        case WalkPhase::FALLING:
            return "FALLING";
        default:
            return "UNKNOWN";
    }
}

inline std::ostream& operator<<(std::ostream& os, WalkPhase walkPhase) {
    os << toString(walkPhase);
    return os;
}

}  // namespace ismpc
