#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <iostream>

#include "ismpc_cpp/modules/walk_engine.h"

namespace nb = nanobind;
using EigenMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

namespace ismpc {
namespace python {

NB_MODULE(ismpc_py, m) {
    nb::class_<WalkEngine> WalkEngine(m, "WalkEngine");
    WalkEngine.def(nb::init<>())
        .def("update", &WalkEngine::update)
        .def("get_footsteps", &WalkEngine::get_footsteps)
        .def("get_reference", &WalkEngine::get_reference)
        .def("get_state", &WalkEngine::get_state)
        .def("get_walk_state", &WalkEngine::get_walk_state)
        .def("get_frame_info", &WalkEngine::get_frame_info)
        .def("set_reference_velocity", &WalkEngine::set_reference_velocity)
        .def("get_history", &WalkEngine::get_history)
        .def("get_walk_history", &WalkEngine::get_walk_history)
        .def("get_footstep_history", &WalkEngine::get_footstep_history)
        .def("get_timestamp_history", &WalkEngine::get_timestamp_history);

    nb::class_<EndEffector>(m, "EndEffector")
        .def(nb::init<>())
        .def_ro("pose", &EndEffector::pose)
        .def_ro("vel", &EndEffector::vel)
        .def_ro("acc", &EndEffector::acc)
        .def("__str__", &EndEffector::toString);

    nb::class_<Pose2>(m, "Pose2")
        .def(nb::init<>())
        .def("rotation", [](const Pose2 &pose) { return static_cast<double>(pose.rotation); })
        .def_ro("translation", &Pose2::translation);

    nb::class_<Pose3>(m, "Pose3")
        .def(nb::init<>())
        .def_ro("rotation", &Pose3::rotation)
        .def_ro("translation", &Pose3::translation);

    nb::class_<State>(m, "State")
        .def(nb::init<>())
        .def_ro("com", &State::com)
        .def_ro("zmp_pos", &State::zmp_pos)
        .def_ro("zmp_vel", &State::zmp_vel)
        .def_ro("left_foot", &State::left_foot)
        .def_ro("right_foot", &State::right_foot)
        .def("__str__", &State::toString);

    nb::class_<WalkState>(m, "WalkState")
        .def_ro("current_footstep_timestamp", &WalkState::current_footstep_timestamp)
        .def_ro("support_foot_type", &WalkState::support_foot_type)
        .def("__str__", &WalkState::toString);

    nb::class_<FootstepsPlan>(m, "Footsteps")
        .def(nb::init<>())
        .def_ro("timestamps", &FootstepsPlan::timestamps)
        .def_ro("num_predicted_footsteps", &FootstepsPlan::num_predicted_footsteps)
        .def_ro("num_controlled_footsteps", &FootstepsPlan::num_controlled_footsteps)
        .def_ro("theta", &FootstepsPlan::theta)
        .def_ro("x", &FootstepsPlan::x)
        .def_ro("y", &FootstepsPlan::y)
        .def_ro("zmp_midpoints", &FootstepsPlan::zmp_midpoints);

    nb::class_<Reference>(m, "Reference")
        .def(nb::init<>())
        .def("get_velocity", [](Reference &ref) { return ref.get_velocity().vector; })
        .def("get_trajectory", [](Reference &ref) { return ref.get_trajectory().matrix; });

    nb::class_<FrameInfo>(m, "FrameInfo").def(nb::init<>()).def_ro("tk", &FrameInfo::tk).def_ro("k", &FrameInfo::k);
};

}  // namespace python
}  // namespace ismpc
