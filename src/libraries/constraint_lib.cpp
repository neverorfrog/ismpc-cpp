#include "ismpc_cpp/libraries/constraint_lib.h"

namespace ismpc {

ConstraintLib::ConstraintLib(const FootstepsPlan& footsteps, const FeetLib& feet)
    : footsteps(footsteps), feet(feet) {}

InequalityConstraint ConstraintLib::getThetaConstraint() const {
    int F = footsteps.num_predicted_footsteps;
    // Inequality constraint matrix
    Matrix C = Matrix::Zero(F, F);
    C.block(0, 0, F - 1, F - 1).diagonal(0).setConstant(-1);
    C.diagonal(1).setConstant(1);

    // Upper and lower bounds
    VectorX ub = VectorX::Zero(F);
    VectorX lb = VectorX::Zero(F);
    ub.setConstant(theta_max);
    lb.setConstant(-theta_max);

    return InequalityConstraint(C, lb, ub);
}

InequalityConstraint ConstraintLib::getKinematicConstraint(int F) const {
    VectorX cos_theta = footsteps.theta.array().cos();
    VectorX sin_theta = footsteps.theta.array().sin();

    Matrix C = Matrix::Zero(2 * F, 2 * F);
    Matrix Cxj = Matrix::Zero(2, 2);
    Matrix Cyj = Matrix::Zero(2, 2);
    VectorX lb = VectorX::Zero(2 * F);
    VectorX ub = VectorX::Zero(2 * F);
    VectorX lbj = VectorX::Zero(2);
    VectorX ubj = VectorX::Zero(2);

    Pose2 sf_pose = feet.getSupportFootPose().getPose2();
    Scalar current_x = sf_pose.translation(0);
    Scalar current_y = sf_pose.translation(1);

    for (int j = 0; j < F; ++j) {
        if (j == 0) {
            C.block(0, 0, 2, 1) << cos_theta(j), -sin_theta(j);
            C.block(0, F, 2, 1) << sin_theta(j), cos_theta(j);

            Scalar oriented_current_x = cos_theta(j) * current_x + sin_theta(j) * current_y;
            Scalar oriented_current_y = -sin_theta(j) * current_x + cos_theta(j) * current_y;
            lbj << oriented_current_x - 0.5 * dax, oriented_current_y + feet.getFootstepSign(j) * l - 0.5 * day;
            ubj << oriented_current_x + 0.5 * dax, oriented_current_y + feet.getFootstepSign(j) * l + 0.5 * day;
        } else {
            Cxj << -cos_theta(j), cos_theta(j), sin_theta(j), -sin_theta(j);
            Cyj << -sin_theta(j), sin_theta(j), -cos_theta(j), cos_theta(j);

            C.block(2 * j, j - 1, 2, 2) = Cxj;
            C.block(2 * j, (j - 1) + F, 2, 2) = Cyj;

            lbj << -0.5 * dax, feet.getFootstepSign(j) * l - 0.5 * day;
            ubj << 0.5 * dax, feet.getFootstepSign(j) * l + 0.5 * day;
        }

        lb.segment(2 * j, 2) = lbj;
        ub.segment(2 * j, 2) = ubj;
    }

    return InequalityConstraint(C, lb, ub);
}

InequalityConstraint ConstraintLib::getZmpConstraint(const Vector3& lipx, const Vector3& lipy) const {
    int d = 2 * numC;
    Matrix C = Matrix::Zero(d, d);
    VectorX lb = VectorX::Zero(d);
    VectorX ub = VectorX::Zero(d);

    VectorX theta = footsteps.get_zmp_midpoints_theta();
    VectorX cos_theta = theta.array().cos();
    VectorX sin_theta = theta.array().sin();
    VectorX xf = footsteps.get_zmp_midpoints_x();
    VectorX yf = footsteps.get_zmp_midpoints_y();

    Scalar xz = lipx(2);
    Scalar yz = lipy(2);

    for (int i = 0; i < numC; ++i) {
        C.block(i, 0, 1, i + 1).setConstant(delta * cos_theta(i));
        C.block(i, numC, 1, i + 1).setConstant(delta * sin_theta(i));
        C.block(i + numC, 0, 1, i + 1).setConstant(-delta * sin_theta(i));
        C.block(i + numC, numC, 1, i + 1).setConstant(delta * cos_theta(i));

        Scalar x_displacement = (xf(i) - xz);
        Scalar y_displacement = (yf(i) - yz);
        Scalar rotated_x_displacement = cos_theta(i) * x_displacement + sin_theta(i) * y_displacement;
        Scalar rotated_y_displacement = -sin_theta(i) * x_displacement + cos_theta(i) * y_displacement;

        ub(i) = rotated_x_displacement + 0.5 * dxz;
        ub(i + numC) = rotated_y_displacement + 0.5 * dyz;
        lb(i) = rotated_x_displacement - 0.5 * dxz;
        lb(i + numC) = rotated_y_displacement - 0.5 * dyz;
    }

    return InequalityConstraint(C, lb, ub);
}

InequalityConstraint ConstraintLib::getZmpVelocityConstraint() const {
    int n_in = 2 * numC;
    Matrix C = Matrix::Identity(n_in, n_in);
    VectorX lb = VectorX::Ones(n_in);
    VectorX ub = VectorX::Ones(n_in);
    lb.segment(0, numC) *= -zmp_vx_max;
    lb.segment(numC, numC) *= -zmp_vy_max;
    ub.segment(0, numC) *= zmp_vx_max;
    ub.segment(numC, numC) *= zmp_vy_max;

    return InequalityConstraint(C, lb, ub);
}

EqualityConstraint ConstraintLib::getStabilityConstraint(const Vector3& lipx, const Vector3& lipy) const {
    int Fprime = footsteps.num_controlled_footsteps;
    int d = 2 * numC + 2 * Fprime;
    Matrix A = Matrix::Zero(2, d);
    VectorX b = VectorX::Zero(2);

    // Summation of zmp velocities
    for (int i = 0; i < numC; ++i) {
        A(0, i) = std::exp(-i * delta * eta);
        A(1, i + numC) = std::exp(-i * delta * eta);
    }

    Scalar xu = lipx(0) + lipx(1) / eta;
    Scalar yu = lipy(0) + lipy(1) / eta;

    Scalar temp = eta / (1 - std::exp(-delta * eta));
    b(0) = temp * (xu - lipx(2));
    b(1) = temp * (yu - lipy(2));

    if (tail_type == TailType::PERIODIC) {
        b(0) *= (1 - std::exp(-delta * eta * numC));
        b(1) *= (1 - std::exp(-delta * eta * numC));
    } else if (tail_type == TailType::ANTICIPATIVE) {
        VectorX xf = footsteps.get_zmp_midpoints_x();
        VectorX yf = footsteps.get_zmp_midpoints_y();

        Scalar xdz, ydz, exp_term;
        for (int i = numC; i < numP; ++i) {
            exp_term = std::exp(-i * delta * eta);
            xdz = (xf(i) - xf(i - 1)) / delta;
            ydz = (yf(i) - yf(i - 1)) / delta;

            b(0) -= exp_term * xdz;
            b(1) -= exp_term * ydz;
        }
    }

    return EqualityConstraint(A, b);
}

}  // namespace ismpc
