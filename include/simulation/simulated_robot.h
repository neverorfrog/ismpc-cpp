#pragma once

#include <Eigen/Core>

#include "dart/dart.hpp"
#include "representations/frame_info.h"
#include "representations/lip_robot.h"
#include "tools/config/robot_config.h"
#include "tools/config/config.h"
#include "tools/debug.h"
#include "tools/math/rotation_matrix.h"
#include "types/body_parts.h"
#include "types/math_types.h"
#include "types/support_phase.h"
#include "types/tail_type.h"

namespace ismpc {

class SimulatedRobot : public LipRobot {
   private:
    VectorX initialConfiguration;
    // State inside dart simulator (d stands for dart)
    dart::dynamics::SkeletonPtr d_robot;
    dart::dynamics::BodyNode* d_base;
    dart::dynamics::BodyNode* d_torso;
    dart::dynamics::BodyNode* d_left_foot;
    dart::dynamics::BodyNode* d_right_foot;
    dart::dynamics::BodyNode* d_support_foot;
    dart::dynamics::BodyNode* d_swing_foot;

    // Internal State

    void setInitialConfiguration();
    void updateZmpFromExternalForces();
    Matrix getTorsoAndSwfJacobian(const LipRobot& robot);

    // Utility matrices and vector
    Matrix _Rot_buffer;
    Matrix _R_CoM, _R_swgf;
    VectorX ComVref;
    Eigen::VectorXd desired_pose;
    Eigen::VectorXd current_pose;
    Eigen::MatrixXd Jacobian_tot, PseudoJacobian_tot;
    Eigen::VectorXd dq;

    inline RotationMatrix getRotation(dart::dynamics::BodyNode* body, dart::dynamics::BodyNode* reference_frame) {
        return RotationMatrix(body->getTransform(reference_frame).rotation());
    }

    inline RotationMatrix getRotation(dart::dynamics::BodyNode* body) {
        return RotationMatrix(body->getTransform().rotation());
    }

   public:
    SimulatedRobot() = default;
    ~SimulatedRobot() = default;
    SimulatedRobot(dart::dynamics::SkeletonPtr robot);
    void update_state(LipRobot& robot, FrameInfo& frame_info);
    void control(const LipRobot& robot, dart::simulation::WorldPtr world);
    VectorX getJointVelocitiesStacked(const LipRobot& robot);
    void ArmSwing(const LipRobot& robot, dart::simulation::WorldPtr world);
    void FixWaistToChestJoints();
};

}  // namespace ismpc
