#include "ismpc_cpp/libraries/cost_lib.h"

namespace ismpc {

CostLib::CostLib(const Reference& reference, const FootstepsPlan& footsteps, const LipRobot& robot)
    : reference(reference), footsteps(footsteps), robot(robot) {}

Cost CostLib::getThetaCost() const {
    int F = footsteps.num_predicted_footsteps;
    VectorX delta_theta = VectorX::Zero(F);
    Scalar t_start = robot.walk.current_footstep_timestamp;
    Scalar t_end;
    for (int j = 0; j < F; ++j) {
        t_end = footsteps.timestamps[j];
        delta_theta(j) = reference.integrateOmega(t_start, t_end);
        t_start = t_end;
    }
    Scalar current_theta = robot.getSupportFootPose().rotation(2);
    // Cost Matrix
    Matrix H = Matrix::Identity(F, F);
    H.diagonal(0) = 4 * Matrix::Ones(F, 1);
    H.diagonal(1).setConstant(-2);
    H.diagonal(-1).setConstant(-2);
    H(F - 1, F - 1) = 2;

    // Cost vector
    VectorX g = VectorX::Zero(F);
    for (int j = 0; j < F; ++j) {
        if (j == 0) {
            g(j) = 2 * (delta_theta(1) - delta_theta(0) - current_theta);
        } else if (j == F - 1) {
            g(j) = -2 * delta_theta(j);
        } else if (j < F - 1) {
            g(j) = 2 * (delta_theta(j + 1) - delta_theta(j));
        }
    }

    return Cost(H, g);
}

Cost CostLib::getPositionCost() const {
    int F = footsteps.num_predicted_footsteps;

    // Oriented Displacements and Integrated Theta
    VectorX delta_x = VectorX::Zero(F);
    VectorX delta_y = VectorX::Zero(F);
    Scalar integrated_theta = robot.getSupportFootPose().rotation(2);
    Scalar t_start = robot.walk.current_footstep_timestamp;
    Scalar t_end;
    Pose2 displacement;
    for (int j = 0; j < F; ++j) {
        t_end = footsteps.timestamps[j];
        displacement = reference.integrateVelocity(t_start, t_end, integrated_theta);
        t_start = t_end;
        Scalar optimal_theta = footsteps.theta(j);
        delta_x(j) = displacement.translation(0) + robot.getFootstepSign(j) * (-sin(optimal_theta)) * l;
        delta_y(j) = displacement.translation(1) + robot.getFootstepSign(j) * (cos(optimal_theta)) * l;
        integrated_theta = displacement.rotation;
    }

    // Cost Matrix
    Matrix H = Matrix::Zero(2 * F, 2 * F);
    Matrix Hx = Matrix::Identity(F, F);
    Hx.diagonal(0) = 4 * Matrix::Ones(F, 1);
    Hx.diagonal(1).setConstant(-2);
    Hx.diagonal(-1).setConstant(-2);
    Hx(F - 1, F - 1) = 2;
    H.block(0, 0, F, F) = Hx;
    H.block(F, F, F, F) = Hx;

    Pose2 sf_pose = robot.getSupportFootPose().getPose2();
    Scalar current_x = sf_pose.translation(0);
    Scalar current_y = sf_pose.translation(1);

    // Cost vector
    VectorX g = VectorX::Zero(2 * F);
    for (int j = 0; j < F; ++j) {
        if (j == 0) {
            g(j) = 2 * (delta_x(1) - delta_x(0) - current_x);
            g(j + F) = 2 * (delta_y(1) - delta_y(0) - current_y);
        } else if (j == F - 1) {
            g(j) = -2 * delta_x(j);
            g(j + F) = -2 * delta_y(j);
        } else if (j < F - 1) {
            g(j) = 2 * (delta_x(j + 1) - delta_x(j));
            g(j + F) = 2 * (delta_y(j + 1) - delta_y(j));
        }
    }

    return Cost(H, g);
}

Cost CostLib::getMpcCost() const {
    // Cost Matrix
    int Fprime = footsteps.num_controlled_footsteps;
    int d = 2 * numC + 2 * Fprime;
    Matrix H = Matrix::Identity(d, d);
    Matrix H1 = Matrix::Identity(2 * numC, 2 * numC) * 2;             // Square Sum of xdz and ydz
    Matrix H2 = Matrix::Identity(2 * Fprime, 2 * Fprime) * 2 * beta;  // Square Sum of xf and yf
    H.block(0, 0, 2 * numC, 2 * numC) = H1;
    H.block(2 * numC, 2 * numC, 2 * Fprime, 2 * Fprime) = H2;

    // Cost vector
    VectorX g = VectorX::Zero(d);
    for (int i = 2 * numC, j = 0; j < Fprime; ++i, ++j) {
        g(i) = -2 * footsteps.x(j) * beta;
        g(i + Fprime) = -2 * footsteps.y(j) * beta;
    }

    return Cost(H, g);
}

}  // namespace ismpc
