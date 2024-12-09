#pragma once

#include <vector>

#include "representations/footsteps.h"
#include "representations/lip_robot.h"
#include "representations/reference.h"
#include "tools/config/config.h"
#include "tools/config/robot_config.h"
#include "types/math_types.h"
#include "types/optimization.h"

namespace ismpc {

class CostLib {
   private:
    const Reference& reference;
    const FootstepsPlan& footsteps;
    const LipRobot& robot;

    Scalar current_support_x;
    Scalar current_support_y;
    Scalar current_support_theta;

    const int numC = Config::C;  // Number of control steps
    const Scalar l = RobotConfig::l;
    const Scalar beta = RobotConfig::beta;

   public:
    CostLib(const Reference& reference, const FootstepsPlan& footsteps, const LipRobot& robot);

    /**
     * @brief Get the Theta Cost object such as to minimize the squared error between
     * the difference of two adjacent decision variables theta_j, theta_(j+1) and the
     * theta displacement made by a unicycle template model in a certain
     * time interval decided by the reference velocity
     *
     * @return Cost
     */
    Cost getThetaCost() const;

    /**
     * @brief Get the Position Cost object such as to minimize the squared error
     * between the difference of two adjacent variables x_j, x_(j+1) and the position
     * displacement made by a unicycle template model in a certain time interval
     * decived by the reference velocity
     *
     * @return Cost
     */
    Cost getPositionCost() const;

    /**
     * @brief Get the Mpc Cost object such as to minimize the squared sum of zmp velocities
     * and the squared errore between proposed footsteps by the planner and dfootsteps
     * treated as decision variables
     *
     * @return Cost
     */
    Cost getMpcCost() const;
};

}  // namespace ismpc
