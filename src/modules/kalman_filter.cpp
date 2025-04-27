#include "ismpc_cpp/modules/kalman_filter.h"

#include <iostream>

#include "ismpc_cpp/types/configs.h"

namespace ismpc {

KalmanFilter::KalmanFilter(const Params& params)
    : eta2(params.lip.eta * params.lip.eta), delta(params.mpc.delta) {
    Matrix3 lip_A = Matrix3({{0, 1, 0}, {eta2, 0, -eta2}, {0, 0, 0}});
    Vector3 lip_B = Vector3({0, 0, 1});
    Matrix3 A_half = Matrix3::Identity() + lip_A * delta;  // Linear Inverted Pendulum (LIP) model
    Vector3 B_half = lip_B * delta;                        // Maps ZMP velocity inputs to state changes
    Matrix3 H_half = Matrix3::Identity();  // Measurement matrix (all variables are measurable)
    Matrix3 Q_half = Matrix3::Identity();  // Process noise covariance (independent noise)
    Matrix3 R_half = Vector3({1e1, 1e2, 1e4}).asDiagonal();  // Measurement noise covariance
    Matrix3 P_half = Matrix3::Identity();                    // Covariance matrix (initially unknown)

    // Fill the matrices
    A << A_half, Matrix3::Zero(), Matrix3::Zero(), A_half;
    B << B_half, Vector3::Zero(), Vector3::Zero(), B_half;
    H << H_half, Matrix3::Zero(), Matrix3::Zero(), H_half;
    Q << Q_half, Matrix3::Zero(), Matrix3::Zero(), Q_half;
    R << R_half, Matrix3::Zero(), Matrix3::Zero(), R_half;
    P << P_half, Matrix3::Zero(), Matrix3::Zero(), P_half;
    x = Vector6::Zero();  // State vector (initially unknown)
}

void KalmanFilter::update(State& state) {
    // Predict
    x = A * x + B * state.lip.zmp_vel.segment<2>(0);
    P = A * P * A.transpose() + Q;

    // Update
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x = x + K * (state.lip.getState() - H * x);
    P = (Matrix6::Identity() - K * H) * P;

    // Update the state
    state.lip.com_pos(0) = x(0);
    state.lip.com_pos(1) = x(3);
    state.lip.com_vel(0) = x(1);
    state.lip.com_vel(1) = x(4);
    state.lip.zmp_pos(0) = x(2);
    state.lip.zmp_pos(1) = x(5);
}

}  // namespace ismpc
