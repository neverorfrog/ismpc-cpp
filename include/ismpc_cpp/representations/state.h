#pragma once

#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/tools/math/pose2.h"
#include "ismpc_cpp/types/body_parts.h"
#include "ismpc_cpp/types/end_effector.h"
#include "ismpc_cpp/types/footstep.h"
#include "ismpc_cpp/types/lip_state.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/support_phase.h"

namespace ismpc {

struct State {
    // Current state
    LipState lip{};
    EndEffector left_foot{};
    EndEffector right_foot{};
    EndEffector torso{};
    EndEffector base{};  // TODO ??????????

    // Walk Stuff
    Footstep footstep{};
    Pose2 previous_sf_pose{};
    Foot previous_support_foot;
    SupportPhase support_phase;

    // Desired state
    LipState desired_lip{};
    EndEffector desired_left_foot{};
    EndEffector desired_right_foot{};
    EndEffector desired_torso{};
    EndEffector desired_base{};

    // Variables from config to save permanently
    Scalar left_foot_x = RobotConfig::left_foot_x;
    Scalar left_foot_y = RobotConfig::left_foot_y;
    Scalar right_foot_x = RobotConfig::right_foot_x;
    Scalar right_foot_y = RobotConfig::right_foot_y;

    // Time related stuff
    Scalar total_mpc_qp_duration = 0.0;
    Scalar total_mpc_preprocessing_duration = 0.0;
    Scalar total_mpc_postprocessing_duration = 0.0;

    // Plotting related stuff
    std::vector<Footstep> fs_history;
    std::vector<LipState> lip_history;
    std::vector<EndEffector> left_foot_history;
    std::vector<EndEffector> right_foot_history;

    State();

    /**
     * @brief Compute the sign corresponding to the footstep index
     * @param j
     * @return int
     */
    int getFootstepSign(int j) const;

    /**
     * @brief Get the Support Foot Pose object
     */
    const EndEffector& getSupportFoot() const;

    /**
     * @brief Get the Swing Foot Pose object
     */
    const EndEffector& getSwingFoot() const;

    void setSwingFootPose(const Pose2& pose);

    void setDesiredSwingFoot(const EndEffector& foot);

    /**
     * @brief Convert the state to a string representation
     *
     * @return std::string The string representation of the state
     */
    std::string toString() const;

    /**
     * @brief Output stream operator for the State class
     *
     * @param os The output stream
     * @param state The State object to be printed
     * @return std::ostream& The output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const State& state);
};

}  // namespace ismpc
