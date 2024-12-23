#include "ismpc_cpp/representations/state.h"

namespace ismpc {

State::State() {
    left_foot.pose.translation << left_foot_x, left_foot_y, 0;
    right_foot.pose.translation << right_foot_x, right_foot_y, 0;

    Scalar com_x = left_foot_x + 0.5 * (right_foot_x - left_foot_x);
    Scalar com_y = left_foot_y + 0.5 * (right_foot_y - left_foot_y);
    com.pose.translation << com_x, com_y, h;

    zmp_pos << com_x, com_y, 0;
    zmp_vel << 0, 0, 0;

    cosh = std::cosh(eta * delta);
    sinh = std::sinh(eta * delta);
    A << cosh, sinh / eta, 1 - cosh, eta * sinh, cosh, -eta * sinh, 0., 0., 1.;
    B << delta - sinh / eta, 1 - cosh, delta;
}

const Vector3 State::getLipx() const {
    Vector3 lip_x{};
    lip_x << com.pose.translation(0), com.vel(0), zmp_pos(0);
    return lip_x;
}

const Vector3 State::getLipy() const {
    Vector3 lip_y{};
    lip_y << com.pose.translation(1), com.vel(1), zmp_pos(1);
    return lip_y;
}

const Vector3 State::getNextLipx(Scalar xdz) const {
    Vector3 predicted_x = A * getLipx() + B * xdz;
    return predicted_x;
}

const Vector3 State::getNextLipy(Scalar ydz) const {
    Vector3 predicted_y = A * getLipy() + B * ydz;
    return predicted_y;
}

std::string State::toString() const {
    std::ostringstream oss;
    oss << "Center of Mass: \n" << com.toString() << std::endl;
    oss << "ZMP Position: " << zmp_pos.transpose() << std::endl;
    oss << "ZMP Velocity: " << zmp_vel.transpose() << std::endl;
    oss << "Left Foot: \n" << left_foot.toString() << std::endl;
    oss << "Right Foot: \n" << right_foot.toString() << std::endl;
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const State& state) {
    os << state.toString();
    return os;
}

}  // namespace ismpc
