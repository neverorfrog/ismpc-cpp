#include "types/state.h"

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

void State::update(Scalar xdz, Scalar ydz) {
    Vector3 predicted_x = A * getLipx() + B * xdz;
    Vector3 predicted_y = A * getLipy() + B * ydz;
    com.pose.translation << predicted_x(0), predicted_y(0), h;
    com.vel << predicted_x(1), predicted_y(1), 0;
    zmp_pos << predicted_x(2), predicted_y(2), 0;
    com.acc = eta * eta * (com.pose.translation - zmp_pos);
    zmp_vel << xdz, ydz, 0;
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
