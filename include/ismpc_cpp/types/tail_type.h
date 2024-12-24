#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace ismpc {

enum class TailType { TRUNCATED, PERIODIC, ANTICIPATIVE, UNKNOWN };

std::string toString(TailType type);
TailType toTailType(const std::string& str);

}  // namespace ismpc

namespace YAML {
template <>
struct convert<ismpc::TailType> {
    static Node encode(const ismpc::TailType& type);
    static bool decode(const Node& node, ismpc::TailType& type);
};
}  // namespace YAML
