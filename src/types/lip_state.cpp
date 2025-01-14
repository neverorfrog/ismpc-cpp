#include "ismpc_cpp/types/lip_state.h"

namespace ismpc {

Scalar LipState::cosh = std::cosh(RobotConfig::eta * Config::delta);
Scalar LipState::sinh = std::sinh(RobotConfig::eta * Config::delta);
Matrix3 LipState::A = Matrix3({{cosh, sinh / RobotConfig::eta, 1 - cosh},
                               {RobotConfig::eta * sinh, cosh, -RobotConfig::eta* sinh},
                               {0., 0., 1.}});
Vector3 LipState::B = Vector3({Config::delta - sinh / RobotConfig::eta, 1 - cosh, Config::delta});

LipState::LipState() {
    Scalar com_x = RobotConfig::left_foot_x + 0.5 * (RobotConfig::right_foot_x - RobotConfig::left_foot_x);
    Scalar com_y = RobotConfig::left_foot_y + 0.5 * (RobotConfig::right_foot_y - RobotConfig::left_foot_y);
    com_pos << com_x, com_y, RobotConfig::h;
    com_vel << 0, 0, 0;
    com_acc << 0, 0, 0;
    zmp_pos << com_x, com_y, 0;
    zmp_vel << 0, 0, 0;
}

Vector3 LipState::getX() const {
    Vector3 lip_x{};
    lip_x << com_pos(0), com_vel(0), zmp_pos(0);
    return lip_x;
}

Vector3 LipState::getY() const {
    Vector3 lip_y{};
    lip_y << com_pos(1), com_vel(1), zmp_pos(1);
    return lip_y;
}

Vector6 LipState::getState() const {
    Vector6 lip_state{};
    lip_state << getX(), getY();
    return lip_state;
}

Vector3 LipState::integrateX(Scalar xdz) const {
    Vector3 predicted_x = A * getX() + B * xdz;
    return predicted_x;
}

Vector3 LipState::integrateY(Scalar ydz) const {
    Vector3 predicted_y = A * getY() + B * ydz;
    return predicted_y;
}

Vector6 LipState::integrate(Scalar xdz, Scalar ydz) const {
    Vector3 predicted_x = integrateX(xdz);
    Vector3 predicted_y = integrateY(ydz);
    Vector6 predicted_state;
    predicted_state << predicted_x, predicted_y;
    return predicted_state;
}

std::string LipState::toString() const {
    std::ostringstream oss;
    oss << "COM Position: \n" << com_pos.transpose() << std::endl;
    oss << "COM Velocity: \n" << com_vel.transpose() << std::endl;
    oss << "COM Acceleration: \n" << com_acc.transpose() << std::endl;
    oss << "ZMP Position: " << zmp_pos.transpose() << std::endl;
    oss << "ZMP Velocity: " << zmp_vel.transpose() << std::endl;
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const LipState& lip_state) {
    os << lip_state.toString();
    return os;
}

}  // namespace ismpc
