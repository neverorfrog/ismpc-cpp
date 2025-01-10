#include <casadi/casadi.hpp>
#include <core/dm.hpp>
#include <core/mx.hpp>
#include <core/sparsity_interface.hpp>

#include "ismpc_cpp/types/math_types.h"

using casadi::MX, casadi::DM;

namespace ismpc {

class QPSolver {
   public:
    QPSolver() = default;
    QPSolver(int d);

    void init(const DM& H, const DM& g);

    VectorX solve();

   private:
    int d;
    MX H;
    MX g;
    MX x;
    casadi::Opti opti;
};

}  // namespace ismpc
