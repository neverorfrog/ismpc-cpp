#pragma once

#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

struct FrameInfo {
    Scalar tk;
    int k;

    FrameInfo() : tk(0.0), k(0) {}
};

}  // namespace ismpc
