#pragma once

#include <stdexcept>
#include <string>

namespace ismpc {

enum class TailType { TRUNCATED, PERIODIC, ANTICIPATIVE, UNKNOWN };

std::string toString(TailType type);
TailType toTailType(const std::string& str);

}  // namespace ismpc
