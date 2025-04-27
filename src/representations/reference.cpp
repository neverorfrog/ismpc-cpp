#include "ismpc_cpp/representations/reference.h"
#include "ismpc_cpp/types/configs.h"

namespace ismpc {

Reference::Reference(const Params& params): delta(params.mpc.delta) {
    velocity.vx = params.reference.des_vel_x;
    velocity.vy = params.reference.des_vel_y;
    velocity.omega = params.reference.des_omega;
    velocity.vector << params.reference.des_vel_x, params.reference.des_vel_y, params.reference.des_omega;
}

Velocity Reference::get_velocity() const {
    return velocity;
}

Scalar Reference::getVelocityModule() const {
    return std::sqrt(std::pow(velocity.vx, 2) + std::pow(velocity.vy, 2));
}

void Reference::set_velocity(Scalar vx, Scalar vy, Scalar omega) {
    velocity.vx = vx;
    velocity.vy = vy;
    velocity.omega = omega;
    velocity.vector << vx, vy, omega;
}

Scalar Reference::integrateOmega(Scalar start, Scalar end) const {
    Scalar theta = 0;
    Scalar t = start;
    while (t < end) {
        t += delta;
        theta += velocity.omega * delta;
    }
    return theta;
}

Pose2 Reference::integrateVelocity(Scalar start, Scalar end, Scalar current_theta) const {
    Scalar theta = current_theta;
    Scalar x = 0;
    Scalar y = 0;
    Scalar t = start;
    while (t < end) {
        t += delta;
        x += (velocity.vx * std::cos(theta) - velocity.vy * std::sin(theta)) * delta;
        y += (velocity.vx * std::sin(theta) + velocity.vy * std::cos(theta)) * delta;
        theta += velocity.omega * delta;
    }
    return Pose2(theta, x, y);
}

}  // namespace ismpc
