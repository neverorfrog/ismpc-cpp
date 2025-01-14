#include "ismpc_cpp/dart/simulated_robot.h"
#include "ismpc_cpp/tools/math/rotation_matrix.h"

namespace ismpc {

/**
 * @brief This class is responsible for providing the state of the robot.
 * Namely, compute the lip state.
 */
class StateProvider {
   private:
    const SimulatedRobot& robot;

   public:
    StateProvider(const SimulatedRobot& robot);

    void update(State& state);
};

}  // namespace ismpc
