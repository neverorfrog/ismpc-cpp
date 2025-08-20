#include "ismpc_cpp/types/tail_type.h"

namespace ismpc {

std::string toString(TailType type) {
    switch (type) {
        case TailType::TRUNCATED:
            return "TRUNCATED";
        case TailType::PERIODIC:
            return "PERIODIC";
        case TailType::ANTICIPATIVE:
            return "ANTICIPATIVE";
        default:
            return "UNKNOWN";
    }
}

TailType toTailType(const std::string& str) {
    if (str == "TRUNCATED")
        return TailType::TRUNCATED;
    if (str == "PERIODIC")
        return TailType::PERIODIC;
    if (str == "ANTICIPATIVE")
        return TailType::ANTICIPATIVE;
    throw std::runtime_error("Invalid TailType: " + str);
}

}  // namespace ismpc
