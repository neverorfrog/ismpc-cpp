#include <chrono>

#include "modules/walk_engine.h"

int main() {
    ismpc::WalkEngine engine = ismpc::WalkEngine();

    auto start = std::chrono::high_resolution_clock::now();
    for (int k = 0; k < ismpc::Config::N; ++k) {
        PRINT("");
        PRINT("------- k: " << k << "  tk: " << engine.get_frame_info().tk << " -------");
        engine.update();
        engine.print();
        PRINT("---------------------------------");
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    auto mpc_qp_duration = engine.get_robot().total_mpc_qp_duration / ismpc::Config::N;
    auto planner_qp_duration = engine.get_footsteps().total_planner_qp_duration / ismpc::Config::N;
    auto average_duration = duration.count() / ismpc::Config::N;
    auto planner_duration = engine.total_planner_duration / ismpc::Config::N;
    auto mpc_duration = engine.total_mpc_duration / ismpc::Config::N;
    auto mpc_postprocessing_duration = engine.get_robot().total_mpc_postprocessing_duration / ismpc::Config::N;
    auto mpc_preprocessing_duration = engine.get_robot().total_mpc_preprocessing_duration / ismpc::Config::N;

    std::cout << "Average execution time: " << average_duration << " microseconds" << std::endl;
    std::cout << "Average Planner QP execution time: " << planner_qp_duration << " microseconds" << std::endl;
    std::cout << "Average Planner execution time: " << planner_duration << " microseconds" << std::endl;
    std::cout << "Average MPC QP execution time: " << mpc_qp_duration << " microseconds" << std::endl;
    std::cout << "Average MPC execution time: " << mpc_duration << " microseconds" << std::endl;
    std::cout << "Average MPC postprocessing time: " << mpc_postprocessing_duration << " microseconds" << std::endl;
    std::cout << "Average MPC preprocessing time: " << mpc_preprocessing_duration << " microseconds" << std::endl;
    return 0;
}
