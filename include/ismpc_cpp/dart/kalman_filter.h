#include "ismpc_cpp/dart/simulated_robot.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

/**
 * @brief This class is responsible for providing the state of the robot.
 * Namely, filter it.
 */
class KalmanFilter {
   public:
    KalmanFilter(const SimulatedRobot& robot);

    void update(State& state);

   private:
    const SimulatedRobot& robot;

    Scalar eta2;

    Eigen::Matrix<Scalar, 6, 6> A;  // State transition matrix (6x6)
    Eigen::Matrix<Scalar, 6, 2> B;  // Control matrix (6x2)
    Eigen::Matrix<Scalar, 6, 6> H;  // Measurement matrix (6x6)
    Eigen::Matrix<Scalar, 6, 6> Q;  // Process noise covariance (6x6)
    Eigen::Matrix<Scalar, 6, 6> R;  // Measurement noise covariance (6x6)
    Eigen::Matrix<Scalar, 6, 6> P;  // Covariance matrix (6x6)
    Eigen::Matrix<Scalar, 6, 6> K;  // Kalman gain (6x6)
    Vector6 x;                      // State vector (6x1)
};

}  // namespace ismpc
