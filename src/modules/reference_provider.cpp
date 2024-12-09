#include "modules/reference_provider.h"

namespace ismpc {

ReferenceProvider::ReferenceProvider(const FrameInfo& frame_info, const LipRobot& robot)
    : robot(robot), frame_info(frame_info) {}

// TODO for now the velocity is always the same
void ReferenceProvider::update(Reference& reference) {
    Matrix traj = Matrix(3, Config::P);  // Template robot state trajectory [x, y, theta]
    traj.col(0) = robot.state.com.pose.getPose2().getVector();
    Scalar cur_t = frame_info.tk;
    for (int i = 0; i < Config::P - 1; ++i) {
        traj.col(i + 1) = traj.col(i) + Config::delta * f(traj.col(i), reference.get_velocity().vector);
        cur_t += Config::delta;
    }

    reference.set_trajectory(traj);
}

Matrix ReferenceProvider::f(const Matrix& x, const Matrix& u) {
    Scalar theta = static_cast<Scalar>(x(2));
    Matrix R = RotationMatrix::aroundZ(theta);
    return R * u;
}

}  // namespace ismpc
