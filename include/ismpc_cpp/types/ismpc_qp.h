#pragma once

#include <cmath>

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/tools/proxsuite.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/optimization.h"

namespace ismpc {

enum class Coordinate { X = 0, Y = 1 };

class IsmpcQp {
   private:
    const int numC = Config::C;             // number of control points
    const Scalar beta = RobotConfig::beta;  // moving constraint weight
    const Scalar delta = Config::delta;     // time step
    const Scalar eta = RobotConfig::eta;    // sqrt(g/h)
    const Scalar dxz = RobotConfig::dxz;
    const Scalar zmp_vx_max = RobotConfig::zmp_vx_max;
    const TailType tail_type = Config::tail_type;
    Cost cost;
    EqualityConstraint initial_constraint;
    EqualityConstraint model_constraint;
    EqualityConstraint stability_constraint;
    InequalityConstraint zmp_constraint;

    const int nl = 3;                     // number of lip variables (com pos, com vel, zmp pos)
    const int nv = nl + 1;                // number of variables per coordinate (com pos, com vel, zmp pos, zmp vel)
    const int d = nv * numC + nl;         // number of primal variables (xc, xdc, xz, xdz)
    const int n_in = numC + 1;            // number of inequality constraints (zmp pos)
    const int n_eq = nl + nl * numC + 1;  // number of equality constraints (initial, model, stability)
    QP<Scalar> qp = QP<Scalar>(d, n_eq, n_in);
    Matrix A = Matrix(n_eq, d);
    VectorX b = VectorX(n_eq);

    VectorX sol = VectorX::Zero(d);

   public:
    IsmpcQp();

    /**
     * @brief Update the QP problem
     *
     *
     * the Mpc Cost object such as to minimize the squared sum of zmp velocities
     * and the squared errore between proposed footsteps by the planner and dfootsteps
     * treated as decision variables
     *
     * The Zmp Constraint object such as to keep the zmp always inside the convex hull.
     * In single support phase this corresponds to the support foot itself, while in double support
     * it is a moving rectangle (same size of the feet approximately) from the previous
     * support foot to the current one
     */
    void update(const Vector3& lip, const VectorX& mc);

    /**
     * @brief Solve the QP problem
     *
     * @return solution
     */
    VectorX solve();
};

}  // namespace ismpc