#include "ismpc_cpp/dart/state_provider.h"

#include "ismpc_cpp/tools/math/rotation_matrix.h"

namespace ismpc {

void StateProvider::update(SimulatedRobot& robot) {
    State& state = robot.state;

    // Update the com of the robot
    state.lip.com_pos = robot.skeleton->getCOM();
    state.lip.com_vel = robot.skeleton->getCOMLinearVelocity();
    state.lip.com_acc = robot.skeleton->getCOMLinearAcceleration();

    // Left foot
    Eigen::Isometry3d l_foot_transform = robot.d_left_foot->getWorldTransform();
    state.left_foot.pose.translation = l_foot_transform.translation();
    state.left_foot.pose.euler = l_foot_transform.rotation().eulerAngles(0, 1, 2);
    state.left_foot.pose.rotation = RotationMatrix(l_foot_transform.rotation().matrix());
    state.left_foot.lin_vel = robot.d_left_foot->getLinearVelocity();

    // Right foot
    Eigen::Isometry3d r_foot_transform = robot.d_right_foot->getWorldTransform();
    state.right_foot.pose.translation = r_foot_transform.translation();
    state.right_foot.pose.euler = r_foot_transform.rotation().eulerAngles(0, 1, 2);
    state.right_foot.pose.rotation = RotationMatrix(r_foot_transform.rotation().matrix());
    state.right_foot.lin_vel = robot.d_right_foot->getLinearVelocity();

    // Update the ZMP
    bool left_contact = false;
    bool right_contact = false;
    Eigen::VectorXd grf_L = robot.d_left_foot->getConstraintImpulse();
    Eigen::VectorXd grf_R = robot.d_right_foot->getConstraintImpulse();
    Eigen::Vector3d left_cop, right_cop;

    if (abs(grf_L[5]) > 0.01) {
        left_cop << -grf_L(1) / grf_L(5), grf_L(0) / grf_L(5), 0.0;
        left_cop = robot.d_left_foot->getWorldTransform().translation() +
                   robot.d_left_foot->getWorldTransform().rotation() * left_cop;
        left_contact = true;
    }

    if (abs(grf_R[5]) > 0.01) {
        right_cop << -grf_R(1) / grf_R(5), grf_R(0) / grf_R(5), 0.0;
        right_cop = robot.d_right_foot->getWorldTransform().translation() +
                    robot.d_right_foot->getWorldTransform().rotation() * right_cop;
        right_contact = true;
    }

    if (left_contact && right_contact) {
        state.lip.zmp_pos =
            Eigen::Vector3d((left_cop(0) * grf_L[5] + right_cop(0) * grf_R[5]) / (grf_L[5] + grf_R[5]),
                            (left_cop(1) * grf_L[5] + right_cop(1) * grf_R[5]) / (grf_L[5] + grf_R[5]), 0.0);
    } else if (left_contact) {
        state.lip.zmp_pos = Eigen::Vector3d(left_cop(0), left_cop(1), 0.0);
    } else if (right_contact) {
        state.lip.zmp_pos = Eigen::Vector3d(right_cop(0), right_cop(1), 0.0);
    }
}

}  // namespace ismpc
