#include "types/tail_type.h"

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

namespace YAML {

Node convert<ismpc::TailType>::encode(const ismpc::TailType& type) {
    Node node;
    node = ismpc::toString(type);
    return node;
}

bool convert<ismpc::TailType>::decode(const Node& node, ismpc::TailType& type) {
    if (!node.IsScalar()) {
        return false;
    }
    type = ismpc::toTailType(node.as<std::string>());
    return true;
}

}  // namespace YAML
