#include "ismpc_cpp/types/optimization.h"

namespace ismpc {

Cost::Cost(Matrix H, VectorX g) : H(H), g(g) {}

std::ostream& operator<<(std::ostream& os, const Cost& cost) {
    os << "H:\n" << cost.H << "\ng:\n" << cost.g << "\n";
    return os;
}

InequalityConstraint::InequalityConstraint(Matrix C, VectorX l, VectorX u) : C(C), l(l), u(u) {}

std::ostream& operator<<(std::ostream& os, const InequalityConstraint& inequality) {
    Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    os << "C:\n"
       << inequality.C.format(HeavyFmt) << "\nl:\n"
       << inequality.l.format(HeavyFmt) << "\nu:\n"
       << inequality.u.format(HeavyFmt) << "\n";
    return os;
}

EqualityConstraint::EqualityConstraint(Matrix A, VectorX b) : A(A), b(b) {}

std::ostream& operator<<(std::ostream& os, const EqualityConstraint& equality) {
    os << "A:\n" << equality.A << "\nb:\n" << equality.b << "\n";
    return os;
}

}  // namespace ismpc
