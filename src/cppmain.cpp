#include <chrono>

#include "ismpc_cpp/modules/walk_engine.h"
#include "ismpc_cpp/representations/state.h"

int main() {
    ismpc::WalkEngine engine = ismpc::WalkEngine();
    ismpc::State desired_state = ismpc::State();

    auto start = std::chrono::high_resolution_clock::now();
    for (int k = 0; k < ismpc::Config::N; ++k) {
        PRINT("");
        PRINT("------- k: " << k << "  tk: " << engine.get_frame_info().tk << " -------");
        engine.update(desired_state);
        engine.print();
        engine.set_state(desired_state);
        PRINT("---------------------------------");
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    auto planner_qp_duration = engine.get_footsteps().total_planner_qp_duration / ismpc::Config::N;
    auto average_duration = duration.count() / ismpc::Config::N;
    auto planner_duration = engine.total_planner_duration / ismpc::Config::N;
    auto mpc_duration = engine.total_mpc_duration / ismpc::Config::N;

    std::cout << "Average execution time: " << average_duration << " microseconds" << std::endl;
    std::cout << "Average Planner QP execution time: " << planner_qp_duration << " microseconds" << std::endl;
    std::cout << "Average Planner execution time: " << planner_duration << " microseconds" << std::endl;
    std::cout << "Average MPC execution time: " << mpc_duration << " microseconds" << std::endl;
    return 0;
}
