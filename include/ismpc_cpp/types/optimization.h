#pragma once

#include <iostream>

#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

struct Cost {
    Matrix H;
    VectorX g;

    Cost(Matrix H, VectorX g);

    friend std::ostream& operator<<(std::ostream& os, const Cost& cost);
};

struct InequalityConstraint {
    Matrix C;
    VectorX l;
    VectorX u;

    InequalityConstraint(Matrix C, VectorX l, VectorX u);

    friend std::ostream& operator<<(std::ostream& os, const InequalityConstraint& inequality);
};

struct EqualityConstraint {
    Matrix A;
    VectorX b;

    EqualityConstraint(Matrix A, VectorX b);

    friend std::ostream& operator<<(std::ostream& os, const EqualityConstraint& equality);
};

}  // namespace ismpc
