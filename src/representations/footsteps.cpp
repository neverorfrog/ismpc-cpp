#include "representations/footsteps.h"

namespace ismpc {

VectorX FootstepsPlan::get_zmp_midpoints_x() const {
    return VectorX(zmp_midpoints.row(0));
}

VectorX FootstepsPlan::get_zmp_midpoints_y() const {
    return VectorX(zmp_midpoints.row(1));
}

VectorX FootstepsPlan::get_zmp_midpoints_theta() const {
    return VectorX(zmp_midpoints.row(2));
}

Scalar FootstepsPlan::getNextTimestamp() const {
    return timestamps[0];
}

}  // namespace ismpc
