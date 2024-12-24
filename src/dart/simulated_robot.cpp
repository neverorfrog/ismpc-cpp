#include "ismpc_cpp/dart/simulated_robot.h"

namespace ismpc {

SimulatedRobot::SimulatedRobot(dart::dynamics::SkeletonPtr skeleton) : skeleton(skeleton) {
    setInitialConfiguration();
    d_left_foot = skeleton->getBodyNode("l_sole");
    d_right_foot = skeleton->getBodyNode("r_sole");
    d_base = skeleton->getBodyNode("base_link");
    d_torso = skeleton->getBodyNode("torso");
}

void SimulatedRobot::setInitialConfiguration() {
    // floating base
    skeleton->setPosition(0, 0.0);
    skeleton->setPosition(1, -0 * M_PI / 180.0);
    skeleton->setPosition(2, 0.0);
    skeleton->setPosition(3, 0.035502257 + 0.0);
    skeleton->setPosition(4, -0.0);
    skeleton->setPosition(5, 0.751 + 0.00138 - 0.000005);

    // right leg
    skeleton->setPosition(skeleton->getDof("R_HIP_Y")->getIndexInSkeleton(), 0.0);
    skeleton->setPosition(skeleton->getDof("R_HIP_R")->getIndexInSkeleton(), -3 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_HIP_P")->getIndexInSkeleton(), -25 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_KNEE_P")->getIndexInSkeleton(), 50 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_ANKLE_P")->getIndexInSkeleton(), -26 * M_PI / 180 + 0.0175);
    skeleton->setPosition(skeleton->getDof("R_ANKLE_R")->getIndexInSkeleton(), 4 * M_PI / 180 - 0.01745);

    // left leg
    skeleton->setPosition(skeleton->getDof("L_HIP_Y")->getIndexInSkeleton(), 0.0);
    skeleton->setPosition(skeleton->getDof("L_HIP_R")->getIndexInSkeleton(), 3 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_HIP_P")->getIndexInSkeleton(), -25 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_KNEE_P")->getIndexInSkeleton(), 50 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_ANKLE_P")->getIndexInSkeleton(), -26 * M_PI / 180 + 0.0175);
    skeleton->setPosition(skeleton->getDof("L_ANKLE_R")->getIndexInSkeleton(), -4 * M_PI / 180 + 0.01745);

    // right arm
    skeleton->setPosition(skeleton->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4) * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_SHOULDER_R")->getIndexInSkeleton(), -8 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("R_SHOULDER_Y")->getIndexInSkeleton(), 0);
    skeleton->setPosition(skeleton->getDof("R_ELBOW_P")->getIndexInSkeleton(), -25 * M_PI / 180);

    // left arm
    skeleton->setPosition(skeleton->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4) * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_SHOULDER_R")->getIndexInSkeleton(), 8 * M_PI / 180);
    skeleton->setPosition(skeleton->getDof("L_SHOULDER_Y")->getIndexInSkeleton(), 0);
    skeleton->setPosition(skeleton->getDof("L_ELBOW_P")->getIndexInSkeleton(), -25 * M_PI / 180);
}

}  // namespace ismpc
