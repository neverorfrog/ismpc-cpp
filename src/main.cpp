#include <chrono>

#include "ismpc_cpp/ismpc.h"
#include "ismpc_cpp/modules/foot_trajectory_generator.h"
#include "ismpc_cpp/modules/footstep_plan_provider.h"
#include "ismpc_cpp/modules/model_predictive_controller.h"
#include "ismpc_cpp/modules/moving_constraint_provider.h"
#include "ismpc_cpp/modules/reference_provider.h"
#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/reference.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/types/configs.h"

int main() {
    // Configs
    ismpc::Params params = ismpc::Params();

    // Representations
    ismpc::FrameInfo frame_info = ismpc::FrameInfo();
    ismpc::Reference reference = ismpc::Reference(params);
    ismpc::State state = ismpc::State(params);
    ismpc::FootstepPlan plan = ismpc::FootstepPlan(params);

    // Modules
    ismpc::FootstepPlanProvider planner = ismpc::FootstepPlanProvider(frame_info, reference, state, plan, params);
    ismpc::ModelPredictiveController mpc = ismpc::ModelPredictiveController(frame_info, state, plan, params);
    ismpc::FootTrajectoryGenerator ft_generator = ismpc::FootTrajectoryGenerator(frame_info, state, plan, params);
    ismpc::MovingConstraintProvider mc_provider = ismpc::MovingConstraintProvider(frame_info, state, plan, params);

    // Timing stuff
    std::chrono::system_clock::time_point start, end;
    long total_planner_duration = 0.0;
    long total_mpc_duration = 0.0;
    long total_feet_duration = 0.0;

    // Update the footstep planner
    start = std::chrono::high_resolution_clock::now();
    planner.update(plan);
    end = std::chrono::high_resolution_clock::now();
    total_planner_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    for (int k = 0; k < params.mpc.N; ++k) {
        std::cout << "------- k: " << k << "  tk: " << frame_info.tk << " -------" << std::endl;

        // Update the moving constraint provider
        start = std::chrono::high_resolution_clock::now();
        mc_provider.update(plan);
        end = std::chrono::high_resolution_clock::now();

        // Update the MPC module
        start = std::chrono::high_resolution_clock::now();
        mpc.update(state);
        // std::cout << "CURRENT STATE: \n LIP: \n" << state.lip << std::endl;
        // std::cout << "LEFT FOOT: "
        //           << state.left_foot.pose.getPose2().getVector().transpose().format(ismpc::Config::CleanFmt)
        //           << std::endl;
        // std::cout << "RIGHT FOOT: "
        //           << state.right_foot.pose.getPose2().getVector().transpose().format(ismpc::Config::CleanFmt) <<
        //           "\n"
        //           << std::endl;
        // std::cout << "DESIRED LIP: \n" << state.desired_lip << std::endl;
        // std::cout << "FOOTSTEP: \n" << state.footstep << std::endl;
        // std::cout << "MOVING CONSTRAINTS: \n"
        //           << plan.zmp_midpoints_y.transpose().format(ismpc::Config::CleanFmt) << std::endl;
        end = std::chrono::high_resolution_clock::now();
        total_mpc_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        // Generate the foot trajectory
        start = std::chrono::high_resolution_clock::now();
        ft_generator.update(state);
        end = std::chrono::high_resolution_clock::now();
        total_feet_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        // Update the actual state
        state.lip = state.desired_lip;
        state.left_foot = state.desired_left_foot;
        state.right_foot = state.desired_right_foot;

        // Update time
        frame_info.k += 1;
        frame_info.tk += params.mpc.delta;
        std::cout << "---------------------------------\n\n\n" << std::endl;
    }

    auto duration = total_feet_duration + total_mpc_duration + total_planner_duration;
    auto average_duration = duration / params.mpc.N;
    auto average_planner_duration = total_planner_duration / params.mpc.N;
    auto average_mpc_duration = total_mpc_duration / params.mpc.N;
    auto average_feet_duration = total_feet_duration / params.mpc.N;

    std::cout << "Average execution time: " << average_duration << " microseconds" << std::endl;
    std::cout << "Average planner execution time: " << average_planner_duration << " microseconds" << std::endl;
    std::cout << "Average mpc execution time: " << average_mpc_duration << " microseconds" << std::endl;
    std::cout << "Average feet execution time: " << average_feet_duration << " microseconds" << std::endl;
    return 0;
}
