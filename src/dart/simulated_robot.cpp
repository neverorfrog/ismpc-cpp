#include "ismpc_cpp/dart/simulated_robot.h"

#include <dart/math/MathTypes.hpp>
#include <optional>

#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

SimulatedRobot::SimulatedRobot(dart::dynamics::SkeletonPtr skeleton)
    : skeleton(skeleton), qp(skeleton->getNumDofs()) {
    d_left_foot = skeleton->getBodyNode("l_sole");
    d_right_foot = skeleton->getBodyNode("r_sole");
    d_base = skeleton->getBodyNode("body");
    d_torso = skeleton->getBodyNode("torso");

    setInitialConfiguration();
    initial_configuration = skeleton->getPositions();

    std::array<std::string, 8> redundant_dofs = {"NECK_Y",       "NECK_P",       "R_SHOULDER_P", "R_SHOULDER_R",
                                                 "R_SHOULDER_Y", "L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y"};
    joint_selection = Matrix::Zero(skeleton->getNumDofs(), skeleton->getNumDofs());
    for (const auto& dof_name : redundant_dofs) {
        joint_selection(skeleton->getDof(dof_name)->getIndexInSkeleton(),
                        skeleton->getDof(dof_name)->getIndexInSkeleton()) = 1;
    }
}

VectorX SimulatedRobot::getJointRequest(const State& desired) {
    std::cout << "RETRIEVING JOINT REQUEST" << std::endl;

    // Define QP Problem
    int d = skeleton->getNumDofs();
    VectorX qpos = skeleton->getPositions();   // d x 1
    VectorX qvel = skeleton->getVelocities();  // d x 1
    Matrix H = Matrix::Zero(d, d);
    VectorX g = VectorX::Zero(d);
    Matrix J;
    Matrix Jtrans;
    Matrix Jdot;
    VectorX pos_error;
    VectorX vel_error;
    VectorX ff;

    // COM cost
    J = skeleton->getCOMLinearJacobian();                                    // 3 x d
    Jtrans = J.transpose();                                                  // d x 3
    Jdot = skeleton->getCOMLinearJacobianDeriv();                            // 3 x d
    pos_error = desired.com_pos - state.com_pos;                             // 3 x 1
    vel_error = desired.com_vel - state.com_vel;                             // 3 x 1
    ff = desired.com_acc;                                                    // 3 x 1
    H += Jtrans * J;                                                         // d x d
    g += -Jtrans * (-Jdot * qvel + ff + 0.1 * pos_error + 1.0 * vel_error);  // d x 1

    // Left foot cost
    J = skeleton->getJacobian(d_left_foot, dart::dynamics::Frame::World());                 // 6 x d
    Jtrans = J.transpose();                                                                 // d x 6
    Jdot = skeleton->getJacobianClassicDeriv(d_left_foot, dart::dynamics::Frame::World());  // 6 x d
    pos_error = (desired.left_foot.pose - state.left_foot.pose).getVector();                // 6 x 1
    std::cout << "Left Foot Pos Error: " << pos_error.transpose() << std::endl;
    vel_error = desired.left_foot.getVelocity() - state.left_foot.getVelocity();            // 6 x 1
    ff = desired.left_foot.getAcceleration();                                               // 6 x 1
    H += Jtrans * J;                                                                        // d x d
    g += -Jtrans * (-Jdot * qvel + ff + 0.1 * pos_error + 1.0 * vel_error);                 // d x 1

    // Right foot cost
    J = skeleton->getJacobian(d_right_foot, dart::dynamics::Frame::World());                 // 6 x d
    Jtrans = J.transpose();                                                                  // d x 6
    Jdot = skeleton->getJacobianClassicDeriv(d_right_foot, dart::dynamics::Frame::World());  // 6 x d
    pos_error = (desired.right_foot.pose - state.right_foot.pose).getVector();               // 6 x 1
    std::cout << "Right Foot Pos Error: " << pos_error.transpose() << std::endl;
    vel_error = desired.right_foot.getVelocity() - state.right_foot.getVelocity();           // 6 x 1
    ff = desired.right_foot.getAcceleration();                                               // 6 x 1
    H += Jtrans * J;                                                                         // d x d
    g += -Jtrans * (-Jdot * qvel + ff + 0.1 * pos_error + 1.0 * vel_error);                  // d x 1

    // Redundant dofs cost
    pos_error = initial_configuration - qpos;                             // d x 1
    vel_error = -qvel;                                                    // d x 1
    H += 1e-2 * joint_selection;                                          // d x d
    g += 1e-2 * joint_selection * (0.01 * pos_error + 10.0 * vel_error);  // d x 1

    // // Torso cost
    // J = d_torso->getAngularJacobian();
    // Jdot = d_torso->getAngularJacobianDeriv();
    // pos_error = (desired.torso.pose.rotation.inverse() * state.torso.pose.rotation).getRPY();
    // vel_error = desired.torso.ang_vel - state.torso.ang_vel;
    // ff = desired.torso.ang_acc;
    // H += J.transpose() * J;
    // g += J.transpose() * (Jdot * qvel - ff - 1. * pos_error - 1. * vel_error);

    // // Base cost
    // J = d_base->getAngularJacobian();
    // Jdot = d_base->getAngularJacobianDeriv();
    // pos_error = (desired.base.pose.rotation.inverse() * state.base.pose.rotation).getRPY();
    // vel_error = desired.base.ang_vel - state.base.ang_vel;
    // ff = desired.base.ang_acc;
    // H += J.transpose() * J;
    // g += J.transpose() * (Jdot * qvel - ff - 1. * pos_error - 10. * vel_error);

    // // Equality constraint
    // Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, d);
    // Eigen::VectorXd b = Eigen::VectorXd::Zero(6, 1);
    // A = J;
    // b = -Jdot * qvel + 0.1 * pos_error + 10. * vel_error;

    if (!H.allFinite()) {
        throw std::runtime_error("H contains NaN or Inf values");
    }

    if (!g.allFinite()) {
        throw std::runtime_error("g contains NaN or Inf values");
    }

    casadi::DM H_casadi = casadi::DM(d, d);
    casadi::DM g_casadi = casadi::DM(d, 1);
    for (int i = 0; i < d; i++) {
        for (int j = 0; j < d; j++) {
            H_casadi(i, j) = H(i, j);
        }
        g_casadi(i) = g(i);
    }
    qp.init(H_casadi, g_casadi);
    VectorX sol = qp.solve();
    VectorX command = sol.segment(6, d - 6);

    std::cout << "Joint Request: " << command.transpose() << std::endl;

    return command;
}

void SimulatedRobot::setInitialConfiguration() {
    // right leg
    skeleton->setPosition(skeleton->getDof("R_HIP_Y")->getIndexInSkeleton(), 0.0);
    skeleton->setPosition(skeleton->getDof("R_HIP_R")->getIndexInSkeleton(), -3 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_HIP_P")->getIndexInSkeleton(), -25 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_KNEE_P")->getIndexInSkeleton(), 50 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_ANKLE_P")->getIndexInSkeleton(), -25 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_ANKLE_R")->getIndexInSkeleton(), 3 * M_PI / 180);

    // left leg
    skeleton->setPosition(skeleton->getDof("L_HIP_Y")->getIndexInSkeleton(), 0.0);
    skeleton->setPosition(skeleton->getDof("L_HIP_R")->getIndexInSkeleton(), 3 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_HIP_P")->getIndexInSkeleton(), -25 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_KNEE_P")->getIndexInSkeleton(), 50 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_ANKLE_P")->getIndexInSkeleton(), -25 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_ANKLE_R")->getIndexInSkeleton(), -3 * M_PI / 180);

    // right arm
    skeleton->setPosition(skeleton->getDof("R_SHOULDER_P")->getIndexInSkeleton(), 4 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_SHOULDER_R")->getIndexInSkeleton(), -8 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_SHOULDER_Y")->getIndexInSkeleton(), 0);
    skeleton->setPosition(skeleton->getDof("R_ELBOW_P")->getIndexInSkeleton(), -25 * M_PI / 180);

    // left arm
    skeleton->setPosition(skeleton->getDof("L_SHOULDER_P")->getIndexInSkeleton(), 4 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_SHOULDER_R")->getIndexInSkeleton(), 8 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_SHOULDER_Y")->getIndexInSkeleton(), 0);
    skeleton->setPosition(skeleton->getDof("L_ELBOW_P")->getIndexInSkeleton(), -25 * M_PI / 180);

    // Set floating base height
    Vector3 left_foot_pos = d_left_foot->getTransform().translation();
    Vector3 right_foot_pos = d_right_foot->getTransform().translation();
    skeleton->setPosition(5, -(left_foot_pos(2) + right_foot_pos(2)) / 2.0);
}

}  // namespace ismpc
