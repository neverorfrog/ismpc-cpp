#pragma once
#include <string>
namespace ismpc {

enum class SupportPhase { SINGLE, DOUBLE };

inline std::string toString(SupportPhase supportPhase) {
    switch (supportPhase) {
        case SupportPhase::SINGLE:
            return "SINGLE";
        case SupportPhase::DOUBLE:
            return "DOUBLE";
        default:
            return "UNKNOWN";
    }
}

inline std::ostream& operator<<(std::ostream& os, SupportPhase supportPhase) {
    os << toString(supportPhase);
    return os;
}

}  // namespace ismpc
