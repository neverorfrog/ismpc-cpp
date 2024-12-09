#pragma once

#include <cmath>
#include <vector>

#include "representations/footsteps.h"
#include "representations/lip_robot.h"
#include "tools/config/config.h"
#include "tools/config/robot_config.h"
#include "types/math_types.h"
#include "types/optimization.h"
#include "types/tail_type.h"

namespace ismpc {

class ConstraintLib {
   private:
    const FootstepsPlan& footsteps;
    const LipRobot& robot;

    Scalar theta_max = RobotConfig::theta_max;
    Scalar eta = RobotConfig::eta;
    Scalar dax = RobotConfig::dax;
    Scalar day = RobotConfig::day;
    Scalar l = RobotConfig::l;
    Scalar dxz = RobotConfig::dxz;
    Scalar dyz = RobotConfig::dyz;
    Scalar zmp_vx_max = RobotConfig::zmp_vx_max;
    Scalar zmp_vy_max = RobotConfig::zmp_vy_max;
    TailType tail_type = Config::tail_type;

    Scalar delta = Config::delta;
    const int numC = Config::C;  // Number of control steps
    const int numP = Config::P;  // Number of prediction steps

   public:
    ConstraintLib(const FootstepsPlan& footsteps, const LipRobot& robot);

    /**
     * @brief Get the Theta Constraint object to maintain the theta displacement
     * between two adjacent j within a certain limit
     *
     * @return InequalityConstraint
     */
    InequalityConstraint getThetaConstraint() const;

    /**
     * @brief Get the Kinematic Constraint object packed as a struct
     *
     * @param F number of footsteps, whether that we are considering the
     * control horizon or the prediction horizon
     * @return Inequality
     */
    InequalityConstraint getKinematicConstraint(int F) const;

    /**
     * @brief Get the Zmp Constraint object such as to keep the zmp always inside the convex hull.
     * In single support phase this corresponds to the support foot itself, while in double support
     * it is a moving rectangle (same size of the feet approximately) from the previous
     * support foot to the current one
     *
     * @return InequalityConstraint
     */
    InequalityConstraint getZmpConstraint(const Vector3& lipx, const Vector3& lipy) const;

    /**
     * @brief Get the Zmp Velocity Constraint object to keep the zmp velocity within a certain limit
     *
     * @return InequalityConstraint
     */
    InequalityConstraint getZmpVelocityConstraint() const;

    /**
     * @brief Get the Stability Constraint object (TODO doc)
     *
     * @return EqualityConstraint
     */
    EqualityConstraint getStabilityConstraint(const Vector3& lipx, const Vector3& lipy) const;
};

}  // namespace ismpc
