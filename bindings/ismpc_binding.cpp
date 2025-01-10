#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <iostream>

#include "ismpc_cpp/ismpc.h"
#include "ismpc_cpp/modules/casadi_mpc.h"
#include "ismpc_cpp/modules/footstep_plan_provider.h"
#include "ismpc_cpp/types/lip_state.h"

namespace nb = nanobind;
using EigenMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

namespace ismpc {
namespace python {

NB_MODULE(ismpc_py, m) {
    nb::class_<State>(m, "State")
        .def(nb::init<>())
        .def_rw("lip", &State::lip)
        .def_ro("footstep", &State::footstep)
        .def_ro("desired_lip", &State::desired_lip)
        .def_rw("left_foot", &State::left_foot)
        .def_rw("right_foot", &State::right_foot)
        .def_ro("desired_left_foot", &State::desired_left_foot)
        .def_ro("desired_right_foot", &State::desired_right_foot)
        .def_ro("fs_history", &State::fs_history)
        .def_ro("lip_history", &State::lip_history)
        .def_ro("left_foot_history", &State::left_foot_history)
        .def_ro("right_foot_history", &State::right_foot_history)
        .def("__str__", &State::toString);

    nb::class_<FootstepPlanProvider>(m, "FootstepPlanProvider")
        .def(nb::init<const FrameInfo &, const Reference &, const State &, FootstepPlan &>())
        .def("update", &FootstepPlanProvider::update);

    nb::class_<MovingConstraintProvider>(m, "MovingConstraintProvider")
        .def(nb::init<const FrameInfo &, const State &, FootstepPlan &>())
        .def("update", &MovingConstraintProvider::update);

    nb::class_<ModelPredictiveController>(m, "ModelPredictiveController")
        .def(nb::init<const FrameInfo &, const State &, FootstepPlan &>())
        .def("update", &ModelPredictiveController::update);

    nb::class_<FootTrajectoryGenerator>(m, "FootTrajectoryGenerator")
        .def(nb::init<const FrameInfo &, const State &, FootstepPlan &>())
        .def("update", &FootTrajectoryGenerator::update);

    nb::class_<CasadiMPC>(m, "CasadiMPC")
        .def(nb::init<const FrameInfo &, const State &, FootstepPlan &>())
        .def("update", &CasadiMPC::update);

    nb::class_<EndEffector>(m, "EndEffector")
        .def(nb::init<>())
        .def_ro("pose", &EndEffector::pose)
        .def_ro("lin_vel", &EndEffector::lin_vel)
        .def_ro("ang_vel", &EndEffector::ang_vel)
        .def_ro("lin_acc", &EndEffector::lin_acc)
        .def_ro("ang_acc", &EndEffector::ang_acc)
        .def("__str__", &EndEffector::toString);

    nb::class_<Pose2>(m, "Pose2")
        .def(nb::init<>())
        .def("rotation", [](const Pose2 &pose) { return static_cast<double>(pose.rotation); })
        .def_ro("translation", &Pose2::translation)
        .def("__str__", &Pose2::toString);

    nb::class_<Pose3>(m, "Pose3")
        .def(nb::init<>())
        .def_ro("rotation", &Pose3::rotation)
        .def_ro("translation", &Pose3::translation);

    nb::class_<FootstepPlan>(m, "FootstepPlan")
        .def(nb::init<>())
        .def_ro("footsteps", &FootstepPlan::footsteps)
        .def_ro("zmp_midpoints_x", &FootstepPlan::zmp_midpoints_x)
        .def_ro("zmp_midpoints_y", &FootstepPlan::zmp_midpoints_y)
        .def("__str__", &FootstepPlan::toString);

    nb::class_<Reference>(m, "Reference").def(nb::init<>()).def("get_velocity", [](Reference &ref) {
        return ref.get_velocity().vector;
    });

    nb::class_<FrameInfo>(m, "FrameInfo").def(nb::init<>()).def_rw("tk", &FrameInfo::tk).def_rw("k", &FrameInfo::k);

    nb::class_<Footstep>(m, "Footstep")
        .def(nb::init<>())
        .def_ro("start_pose", &Footstep::start_pose)
        .def_ro("end_pose", &Footstep::end_pose)
        .def_ro("timestamp", &Footstep::ds_start)
        .def("__str__", &Footstep::toString);

    nb::class_<LipState>(m, "LipState")
        .def(nb::init<>())
        .def_ro("com_pos", &LipState::com_pos)
        .def_ro("com_vel", &LipState::com_vel)
        .def_ro("com_acc", &LipState::com_acc)
        .def_ro("zmp_pos", &LipState::zmp_pos)
        .def_ro("zmp_vel", &LipState::zmp_vel)
        .def("__str__", &LipState::toString);
};

}  // namespace python
}  // namespace ismpc
