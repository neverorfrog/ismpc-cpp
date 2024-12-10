#include "simulation/simulated_robot.h"

namespace ismpc {

SimulatedRobot::SimulatedRobot(dart::dynamics::SkeletonPtr d_robot) : LipRobot(), d_robot(d_robot) {
  setInitialConfiguration();
  d_left_foot = d_robot->getBodyNode("l_sole");
  d_right_foot = d_robot->getBodyNode("r_sole");
  d_base = d_robot->getBodyNode("base_link");
  d_torso = d_robot->getBodyNode("torso");

  _Rot_buffer = Matrix::Identity(3, 3);
  _R_CoM = Matrix::Identity(6, 6);
  _R_swgf = Eigen::MatrixXd::Identity(6, 6);
}

void SimulatedRobot::update_state(LipRobot& robot, FrameInfo& frame_info) {
  state.com.pose = Pose3(getRotation(d_base), d_robot->getCOM());
  state.com.vel = d_robot->getCOMLinearVelocity();
  state.com.acc = d_robot->getCOMLinearAcceleration();

  updateZmpFromExternalForces();

  state.left_foot.pose = Pose3(getRotation(d_left_foot), d_robot->getCOM());
  state.left_foot.vel = d_left_foot->getCOMLinearVelocity();
  state.left_foot.acc = d_left_foot->getCOMLinearAcceleration();

  state.right_foot.pose = Pose3(getRotation(d_right_foot), d_robot->getCOM());
  state.right_foot.vel = d_right_foot->getCOMLinearVelocity();
  state.right_foot.acc = d_right_foot->getCOMLinearAcceleration();

  walk = robot.walk;

  if (frame_info.k % 30 == 0 && frame_info.k > 100) {
    double gain = 0.075 / 2.0;
    robot.state.com.pose.translation(0) +=
        gain * (state.com.pose.translation(0) - robot.state.com.pose.translation(0));
    robot.state.com.vel(0) += gain * (state.com.vel(0) - robot.state.com.vel(0));
    robot.state.com.pose.translation(1) +=
        gain * (state.com.pose.translation(1) - robot.state.com.pose.translation(1));
    robot.state.com.vel(1) += gain * (state.com.vel(1) - robot.state.com.vel(1));

    robot.state.left_foot.pose.translation +=
        0 * 0.02 * (state.left_foot.pose.translation - robot.state.left_foot.pose.translation);
    robot.state.right_foot.pose.translation +=
        0 * 0.02 * (state.right_foot.pose.translation - robot.state.right_foot.pose.translation);
  }
}

void SimulatedRobot::control(const LipRobot& robot, dart::simulation::WorldPtr world) {
  VectorX qd = getJointVelocitiesStacked(robot);

  PRINT("qd: " << qd.transpose());

  // Set the velocity of the floating d_base to zero
  for (int i = 0; i < 6; ++i) {
    d_robot->setCommand(i, 0);
  }
  // set joint commands
  for (int i = 0; i < 50; ++i) {
    d_robot->setCommand(i + 6, qd(i));  // velocity joint control
  }

  ArmSwing(robot, world);
  // FixWaistToChestJoints();
}

void SimulatedRobot::setInitialConfiguration() {
  VectorX q = d_robot->getPositions();

  // Floating Base
  q[0] = 0.0;
  q[1] = RobotConfig::base_pitch * M_PI / 180;
  q[2] = 0.0;
  q[3] = 0.00;
  q[4] = 0.00;
  q[5] = 0.75;

  d_robot->setPositions(q);

  // right leg
  d_robot->setPosition(d_robot->getDof("R_HIP_Y")->getIndexInSkeleton(), 0.0);
  d_robot->setPosition(d_robot->getDof("R_HIP_R")->getIndexInSkeleton(), RobotConfig::right_hip_roll);
  d_robot->setPosition(d_robot->getDof("R_HIP_P")->getIndexInSkeleton(), RobotConfig::right_hip_pitch);
  d_robot->setPosition(d_robot->getDof("R_KNEE_P")->getIndexInSkeleton(), 50 * M_PI / 180);
  d_robot->setPosition(d_robot->getDof("R_ANKLE_P")->getIndexInSkeleton(), RobotConfig::right_ankle_pitch);
  d_robot->setPosition(d_robot->getDof("R_ANKLE_R")->getIndexInSkeleton(), RobotConfig::right_ankle_roll);

  // left leg
  d_robot->setPosition(d_robot->getDof("L_HIP_Y")->getIndexInSkeleton(), 0.0);
  d_robot->setPosition(d_robot->getDof("L_HIP_R")->getIndexInSkeleton(), RobotConfig::left_hip_roll);
  d_robot->setPosition(d_robot->getDof("L_HIP_P")->getIndexInSkeleton(), RobotConfig::left_hip_pitch);
  d_robot->setPosition(d_robot->getDof("L_KNEE_P")->getIndexInSkeleton(), 50 * M_PI / 180);
  d_robot->setPosition(d_robot->getDof("L_ANKLE_P")->getIndexInSkeleton(), RobotConfig::left_ankle_pitch);
  d_robot->setPosition(d_robot->getDof("L_ANKLE_R")->getIndexInSkeleton(), RobotConfig::left_ankle_roll);

  // Right Arm
  d_robot->setPosition(d_robot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), 4 * M_PI / 180);
  d_robot->setPosition(d_robot->getDof("R_SHOULDER_R")->getIndexInSkeleton(), -8 * M_PI / 180);
  d_robot->setPosition(d_robot->getDof("R_SHOULDER_Y")->getIndexInSkeleton(), 0);
  d_robot->setPosition(d_robot->getDof("R_ELBOW_P")->getIndexInSkeleton(), -20 * M_PI / 180);

  // Left Arm
  d_robot->setPosition(d_robot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), 4 * M_PI / 180);
  d_robot->setPosition(d_robot->getDof("L_SHOULDER_R")->getIndexInSkeleton(), 8 * M_PI / 180);
  d_robot->setPosition(d_robot->getDof("L_SHOULDER_Y")->getIndexInSkeleton(), 0);
  d_robot->setPosition(d_robot->getDof("L_ELBOW_P")->getIndexInSkeleton(), -20 * M_PI / 180);

  // Set initial chest position
  d_robot->setPosition(d_robot->getDof("CHEST_P")->getIndexInSkeleton(), RobotConfig::chest_pitch);
  d_robot->setPosition(d_robot->getDof("CHEST_Y")->getIndexInSkeleton(), RobotConfig::chest_yaw);
}

void SimulatedRobot::updateZmpFromExternalForces() {
  Eigen::Vector3d zmp_v;
  bool left_contact = false;
  bool right_contact = false;

  Eigen::Vector3d left_cop;
  Scalar left_force = abs(d_left_foot->getConstraintImpulse()[5]);
  PRINT("Left foot force: " << left_force);
  if (left_force > 0) {
    left_cop << -d_left_foot->getConstraintImpulse()(1) / d_left_foot->getConstraintImpulse()(5),
        d_left_foot->getConstraintImpulse()(0) / d_left_foot->getConstraintImpulse()(5), 0.0;
    Eigen::Matrix3d iRotation = d_left_foot->getWorldTransform().rotation();
    Eigen::Vector3d iTransl = d_left_foot->getWorldTransform().translation();
    left_cop = iTransl + iRotation * left_cop;
    left_contact = true;
  }

  Eigen::Vector3d right_cop;
  Scalar right_force = abs(d_right_foot->getConstraintImpulse()[5]);
  PRINT("Right foot force: " << right_force);
  if (right_force > 0) {
    right_cop << -d_right_foot->getConstraintImpulse()(1) / d_right_foot->getConstraintImpulse()(5),
        d_right_foot->getConstraintImpulse()(0) / d_right_foot->getConstraintImpulse()(5), 0.0;
    Eigen::Matrix3d iRotation = d_right_foot->getWorldTransform().rotation();
    Eigen::Vector3d iTransl = d_right_foot->getWorldTransform().translation();
    right_cop = iTransl + iRotation * right_cop;
    right_contact = true;
  }

  if (left_contact && right_contact) {
    PRINT("Both feet in contact");
    zmp_v << (left_cop(0) * d_left_foot->getConstraintImpulse()[5] +
              right_cop(0) * d_right_foot->getConstraintImpulse()[5]) /
                 (d_left_foot->getConstraintImpulse()[5] + d_right_foot->getConstraintImpulse()[5]),
        (left_cop(1) * d_left_foot->getConstraintImpulse()[5] +
         right_cop(1) * d_right_foot->getConstraintImpulse()[5]) /
            (d_left_foot->getConstraintImpulse()[5] + d_right_foot->getConstraintImpulse()[5]),
        0.0;
  } else if (left_contact) {
    PRINT("Left foot in contact");
    zmp_v << left_cop(0), left_cop(1), 0.0;
  } else if (right_contact) {
    PRINT("Right foot in contact");
    zmp_v << right_cop(0), right_cop(1), 0.0;
  } else {  // No contact detected
    PRINT("No contact detected");
    zmp_v << 0.0, 0.0, 0.0;
  }

  double k = 0.35;
  double alpha = 0.25;

  zmp_v(0) = zmp_v(0) >= state.zmp_pos(0) + k * RobotConfig::dyz / 2.0 ?
                 zmp_v(0) = state.zmp_pos(0) + k * RobotConfig::dyz / 2.0 :
                 zmp_v(0);
  zmp_v(0) = zmp_v(0) <= state.zmp_pos(0) - k * RobotConfig::dyz / 2.0 ?
                 zmp_v(0) = state.zmp_pos(0) - k * RobotConfig::dyz / 2.0 :
                 zmp_v(0);
  zmp_v(1) = zmp_v(1) >= state.zmp_pos(1) + k * RobotConfig::dyz / 2.0 ?
                 zmp_v(1) = state.zmp_pos(1) + k * RobotConfig::dyz / 2.0 :
                 zmp_v(1);
  zmp_v(1) = zmp_v(1) <= state.zmp_pos(1) - k * RobotConfig::dyz / 2.0 ?
                 zmp_v(1) = state.zmp_pos(1) - k * RobotConfig::dyz / 2.0 :
                 zmp_v(1);

  zmp_v(0) = state.zmp_pos(0) + alpha * (zmp_v(0) - state.zmp_pos(0));
  zmp_v(1) = state.zmp_pos(1) + alpha * (zmp_v(1) - state.zmp_pos(1));

  state.zmp_pos = zmp_v;
}

VectorX SimulatedRobot::getJointVelocitiesStacked(const LipRobot& robot) {
  const State& desired = robot.state;

  ComVref = Eigen::VectorXd::Zero(12);
  Vector3 des_com_vel = desired.com.vel;
  ComVref << 0.0, 0.0, 0.0, des_com_vel(0), des_com_vel(1), des_com_vel(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  desired_pose = Eigen::VectorXd::Zero(12);
  desired_pose << robot.getRelComPose().getVector(), robot.getRelSwingFootPose().getVector();

  // Assemble actual positions and orientations
  current_pose = Eigen::VectorXd::Zero(12);
  current_pose << getRelComPose().getVector(), getRelSwingFootPose().getVector();

  // Get the proper jacobian and pseudoinvert it
  Jacobian_tot = getTorsoAndSwfJacobian(robot);
  PseudoJacobian_tot = (Jacobian_tot.transpose()) * (Jacobian_tot * Jacobian_tot.transpose()).inverse();

  Eigen::VectorXd qDot(50);
  qDot = PseudoJacobian_tot * (ComVref + RobotConfig::ik_gain * RobotConfig::task_gain * (desired_pose - current_pose));

  dq = d_robot->getVelocities();

  return (qDot - dq.segment(6, 50)) / Config::delta;
}

Matrix SimulatedRobot::getTorsoAndSwfJacobian(const LipRobot& robot) {
  if (robot.walk.support_foot_type == Foot::right) {
    d_support_foot = d_robot->getBodyNode("r_sole");
    d_swing_foot = d_robot->getBodyNode("l_sole");
  } else if (robot.walk.support_foot_type == Foot::left) {
    d_support_foot = d_robot->getBodyNode("l_sole");
    d_swing_foot = d_robot->getBodyNode("r_sole");
  }

  Matrix jacobian_to_base = d_robot->getJacobian(d_torso, d_support_foot) -
                            d_robot->getJacobian(d_support_foot, d_support_foot);  //(mBase,mSupportFoot)

  Vector3 com_orientation = robot.state.com.pose.rotation.getRPY();
  _Rot_buffer << 1.0, 0.0, sin(com_orientation(1)), 0.0, cos(com_orientation(0)),
      -cos(com_orientation(1)) * sin(com_orientation(0)), 0.0, sin(com_orientation(0)),
      -sin(com_orientation(1)) * cos(com_orientation(0));

  _R_CoM.block(0, 0, 3, 3) << _Rot_buffer.inverse();

  jacobian_to_base = _R_CoM * jacobian_to_base;

  for (unsigned int i = 0; i < 44; i++) {
    if (i != 5 || i != 6)
      jacobian_to_base.col(i).setZero();
  }

  Eigen::MatrixXd jacobian_to_swing =
      d_robot->getJacobian(d_swing_foot, d_support_foot) - d_robot->getJacobian(d_support_foot, d_support_foot);

  Vector3 sf_orientation = getRotation(d_swing_foot).getRPY();
  Scalar angleX = sf_orientation(0);
  Scalar angleY = sf_orientation(1);
  _Rot_buffer << 1.0, 0.0, sin(angleY), 0.0, cos(angleX), -cos(angleY) * sin(angleX), 0.0, sin(angleX),
      -sin(angleY) * cos(angleX);
  _R_swgf.block(0, 0, 3, 3) << _Rot_buffer.inverse();
  jacobian_to_swing = _R_swgf * jacobian_to_swing;

  Eigen::MatrixXd Jacobian_tot_(12, 56);

  Jacobian_tot_ << jacobian_to_base, jacobian_to_swing;

  Eigen::MatrixXd Jacobian_tot(12, 50);

  // Remove the floating d_base columns
  Jacobian_tot = Jacobian_tot_.block<12, 50>(0, 6);

  return Jacobian_tot;
}

void SimulatedRobot::ArmSwing(const LipRobot& robot, dart::simulation::WorldPtr world) {
  if (world->getSimFrames() > Config::W) {
    Scalar t_start = robot.walk.current_footstep_timestamp;
    Scalar t_end = robot.walk.next_footstep_timestamp;
    Scalar single_support_duration = (t_end - t_start) * RobotConfig::ss_percentage;
    Scalar double_support_duration = (t_end - t_start) * RobotConfig::ds_percentage;

    d_robot->setPosition(d_robot->getDof("R_SHOULDER_P")->getIndexInSkeleton(),
                         (4 - 6 * sin(2 * M_PI * 0.01 * (world->getSimFrames()) /
                                      (-2.0 * (single_support_duration + double_support_duration)))) *
                             M_PI / 180);  // 2.5 amplitude
    d_robot->setPosition(d_robot->getDof("L_SHOULDER_P")->getIndexInSkeleton(),
                         (4 + 6 * sin(2 * M_PI * 0.01 * (world->getSimFrames()) /
                                      (-2.0 * (single_support_duration + double_support_duration)))) *
                             M_PI / 180);

    d_robot->setPosition(d_robot->getDof("R_ELBOW_P")->getIndexInSkeleton(),
                         (-30 - 7 * sin(2 * M_PI * 0.01 * (world->getSimFrames()) /
                                        (-2.0 * (single_support_duration + double_support_duration)))) *
                             M_PI / 180);  // 2.5 amplitude
    d_robot->setPosition(d_robot->getDof("L_ELBOW_P")->getIndexInSkeleton(),
                         (-30 + 7 * sin(2 * M_PI * 0.01 * (world->getSimFrames()) /
                                        (-2.0 * (single_support_duration + double_support_duration)))) *
                             M_PI / 180);
  }
}

void SimulatedRobot::FixWaistToChestJoints() {
  double inclination = 0.55;
  d_robot->setPosition(d_robot->getDof("CHEST_P")->getIndexInSkeleton(),
                       d_robot->getPosition(d_robot->getDof("CHEST_P")->getIndexInSkeleton()) -
                           0.05 * (d_robot->getPosition(d_robot->getDof("CHEST_P")->getIndexInSkeleton())) +
                           inclination * 0.009);
}

}  // namespace ismpc
