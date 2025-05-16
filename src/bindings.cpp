#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/operators.h> 

#include <iostream>

#include "ismpc_cpp/ismpc.h"
#include "ismpc_cpp/tools/math/rotation_matrix.h"
#include "ismpc_cpp/types/footstep.h"
#include "ismpc_cpp/types/lip_state.h"
#include "ismpc_cpp/types/support_phase.h"

namespace nb = nanobind;
using EigenMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
NB_MAKE_OPAQUE(ismpc::RotationMatrix); // Needed for RotationMatrix bindings

namespace ismpc {
namespace python {

NB_MODULE(ismpc, m) {
    m.doc() = "Python bindings for the ISMPC C++ library";

    // --- Enum Bindings ---
    nb::enum_<SupportPhase>(m, "SupportPhase")
        .value("SINGLE", SupportPhase::SINGLE)
        .value("DOUBLE", SupportPhase::DOUBLE)
        .export_values();

    nb::enum_<Foot>(m, "Foot").value("LEFT", Foot::left).value("RIGHT", Foot::right).export_values();

    nb::enum_<TailType>(m, "TailType")
        .value("PERIODIC", TailType::PERIODIC)
        .value("TRUNCATED", TailType::TRUNCATED)
        .value("ANTICIPATIVE", TailType::ANTICIPATIVE)
        .export_values();

    // --- Config Struct Bindings ---
    nb::class_<MpcParams>(m, "MpcParams")
        .def(nb::init<>())
        .def_rw("delta", &MpcParams::delta)
        .def_rw("N", &MpcParams::N)
        .def_rw("P", &MpcParams::P)
        .def_rw("C", &MpcParams::C)
        .def_rw("T_p", &MpcParams::T_p)
        .def_rw("T_c", &MpcParams::T_c)
        .def_rw("beta", &MpcParams::beta)
        .def_rw("tail_type", &MpcParams::tail_type)
        .def_rw("nl", &MpcParams::nl);

    nb::class_<LipParams>(m, "LipParams")
        .def(nb::init<>())
        .def_rw("h", &LipParams::h)
        .def_rw("g", &LipParams::g)
        .def_rw("eta", &LipParams::eta)
        .def_rw("dxz", &LipParams::dxz)
        .def_rw("dyz", &LipParams::dyz)
        .def_rw("zmp_vx_max", &LipParams::zmp_vx_max)
        .def_rw("zmp_vy_max", &LipParams::zmp_vy_max);

    nb::class_<InitialFeetParams>(m, "InitialFeetParams")
        .def(nb::init<>())
        .def_rw("lf_x", &InitialFeetParams::lf_x)
        .def_rw("lf_y", &InitialFeetParams::lf_y)
        .def_rw("rf_x", &InitialFeetParams::rf_x)
        .def_rw("rf_y", &InitialFeetParams::rf_y);

    nb::class_<GaitParams>(m, "GaitParams")
        .def(nb::init<>())
        .def_rw("ds_percentage", &GaitParams::ds_percentage)
        .def_rw("ss_percentage", &GaitParams::ss_percentage)
        .def_rw("step_height", &GaitParams::step_height)
        .def_rw("l", &GaitParams::l)
        .def_rw("dax", &GaitParams::dax)
        .def_rw("day", &GaitParams::day)
        .def_rw("theta_max", &GaitParams::theta_max)
        .def_rw("T_bar", &GaitParams::T_bar)
        .def_rw("L_bar", &GaitParams::L_bar)
        .def_rw("v_bar", &GaitParams::v_bar)
        .def_rw("alpha", &GaitParams::alpha);

    nb::class_<ReferenceParams>(m, "ReferenceParams")
        .def(nb::init<>())
        .def_rw("des_vel_x", &ReferenceParams::des_vel_x)
        .def_rw("des_vel_y", &ReferenceParams::des_vel_y)
        .def_rw("des_omega", &ReferenceParams::des_omega);

    nb::class_<Params>(m, "Params")
        .def(nb::init<>())
        .def_rw("mpc", &Params::mpc)
        .def_rw("lip", &Params::lip)
        .def_rw("initial_feet", &Params::initial_feet)
        .def_rw("gait", &Params::gait)
        .def_rw("reference", &Params::reference);

    // --- Utility/Data Structure Bindings ---
    nb::class_<EndEffector>(m, "EndEffector")
        .def(nb::init<>())
        .def(nb::init<const EndEffector &>())
        .def("__copy__", [](const EndEffector &self) {
            return new EndEffector(self);
        }, nb::rv_policy::take_ownership)
        .def_rw("pose", &EndEffector::pose)
        .def_rw("lin_vel", &EndEffector::lin_vel)
        .def_rw("ang_vel", &EndEffector::ang_vel)
        .def_rw("lin_acc", &EndEffector::lin_acc)
        .def_rw("ang_acc", &EndEffector::ang_acc)
        .def("getVelocity", &EndEffector::getVelocity)
        .def("getPose2", &EndEffector::getPose2)
        .def("__str__", &EndEffector::toString);

    nb::class_<Pose2>(m, "Pose2")
        .def(nb::init<>())
        .def(nb::init<const Vector2 &>())
        .def(nb::init<const Angle &, const Vector2 &>())
        .def(nb::init<const Angle &, const Scalar, const Scalar &>())
        .def(nb::init<const Pose2 &>())
        .def("__copy__", [](const Pose2 &self) {
            return new Pose2(self);
        }, nb::rv_policy::take_ownership)
        .def("rotation", [](const Pose2 &pose) { return static_cast<double>(pose.rotation); })
        .def_ro("translation", &Pose2::translation)
        .def("__str__", &Pose2::toString);

    nb::class_<RotationMatrix>(m, "RotationMatrix")
        .def(nb::init<>())
        .def(nb::init<const EigenMatrix &>())
        .def("__copy__", [](const RotationMatrix &self) {
            return new RotationMatrix(self);
        }, nb::rv_policy::take_ownership)
        .def("matrix", [](const RotationMatrix &r) { return static_cast<EigenMatrix>(r); })
        .def("__mul__", [](const RotationMatrix &r1, const RotationMatrix &r2) { return r1 * r2; })
        .def("getXAngle", &RotationMatrix::getXAngle)
        .def("getYAngle", &RotationMatrix::getYAngle)
        .def("getZAngle", &RotationMatrix::getZAngle)
        .def("getRPY", &RotationMatrix::getRPY)
        .def("__str__", [](const RotationMatrix &r) {
            std::ostringstream oss;
            oss << r;
            return oss.str();
        });

    nb::class_<Pose3>(m, "Pose3")
        .def(nb::init<>())
        .def(nb::init<const RotationMatrix &, const Vector3 &>())
        .def("__copy__", [](const Pose3 &self) {
            return new Pose3(self);
        }, nb::rv_policy::take_ownership)
        .def("getVector", &Pose3::getVector)
        .def("__add__", [](const Pose3 &p1, const Pose3 &p2) { return p1 + p2; })
        .def("__str__", &Pose3::toString)
        .def_rw("rotation", &Pose3::rotation)
        .def_rw("translation", &Pose3::translation);

    nb::class_<Footstep>(m, "Footstep")
        .def(nb::init<>())
        .def_rw("start_pose", &Footstep::start_pose)
        .def_rw("end_pose", &Footstep::end_pose)
        .def_rw("start", &Footstep::start)
        .def_rw("ds_start", &Footstep::ds_start)
        .def_rw("end", &Footstep::end)
        .def_ro("support_foot", &Footstep::support_foot)
        .def("__str__", &Footstep::toString);

    nb::class_<LipState>(m, "LipState")
        // Constructor takes Params
        .def(nb::init<const Params &>())
        .def(nb::init<const LipState &>())
        .def("__copy__", [](const LipState &self) {
            return new LipState(self);
        }, nb::rv_policy::take_ownership)
        .def_rw("com_pos", &LipState::com_pos)
        .def_rw("com_vel", &LipState::com_vel)
        .def_rw("com_acc", &LipState::com_acc)
        .def_rw("zmp_pos", &LipState::zmp_pos)
        .def_rw("zmp_vel", &LipState::zmp_vel)
        .def("getX", &LipState::getX)
        .def("getY", &LipState::getY)
        .def("getState", &LipState::getState)
        .def("integrateX", &LipState::integrateX)
        .def("integrateY", &LipState::integrateY)
        .def("integrate", &LipState::integrate)
        .def("__str__", &LipState::toString);

    nb::class_<FrameInfo>(m, "FrameInfo")
        .def(nb::init<>())
        .def_rw("tk", &FrameInfo::tk)
        .def_rw("k", &FrameInfo::k);

    // --- Representation Bindings ---
    nb::class_<FootstepPlan>(m, "FootstepPlan")
        .def(nb::init<const Params &>())
        .def_rw("footsteps", &FootstepPlan::footsteps) // Changed to rw if modification needed
        .def_rw("zmp_midpoints_x", &FootstepPlan::zmp_midpoints_x) // Changed to rw
        .def_rw("zmp_midpoints_y", &FootstepPlan::zmp_midpoints_y) // Changed to rw
        .def_rw("support_phase", &FootstepPlan::support_phase)
        .def_rw("fs_history", &FootstepPlan::fs_history)
        .def_rw("mc_x_history", &FootstepPlan::mc_x_history) // Changed to rw
        .def_rw("mc_y_history", &FootstepPlan::mc_y_history) // Changed to rw
        .def_rw("mc_theta_history", &FootstepPlan::mc_theta_history) // Changed to rw
        .def_rw("fs_plan_history", &FootstepPlan::fs_plan_history)
        .def("__str__", &FootstepPlan::toString);

    nb::class_<State>(m, "State")
        // Constructor takes Params
        .def(nb::init<const Params &>())
        .def(nb::init<const State &>())
        .def("__copy__", [](const State &self) {
            return new State(self);
        }, nb::rv_policy::take_ownership)
        .def_rw("lip", &State::lip)
        .def_rw("left_foot", &State::left_foot)
        .def_rw("right_foot", &State::right_foot)
        .def_rw("torso", &State::torso)
        .def_rw("base", &State::base)
        .def_rw("desired_lip", &State::desired_lip)
        .def_rw("desired_left_foot", &State::desired_left_foot)
        .def_rw("desired_right_foot", &State::desired_right_foot)
        .def_rw("desired_torso", &State::desired_torso)
        .def_rw("desired_base", &State::desired_base)
        .def_ro("left_foot_x", &State::left_foot_x)
        .def_ro("left_foot_y", &State::left_foot_y)
        .def_ro("right_foot_x", &State::right_foot_x)
        .def_ro("right_foot_y", &State::right_foot_y)
        .def_rw("total_mpc_qp_duration", &State::total_mpc_qp_duration) // Made rw for potential reset
        .def_rw("total_mpc_preprocessing_duration", &State::total_mpc_preprocessing_duration) // Made rw
        .def_rw("total_mpc_postprocessing_duration", &State::total_mpc_postprocessing_duration) // Made rw
        .def_rw("lip_history", &State::lip_history) // Made rw for potential clearing/access
        .def_rw("left_foot_history", &State::left_foot_history) // Made rw
        .def_rw("right_foot_history", &State::right_foot_history) // Made rw
        .def("__str__", &State::toString);

    nb::class_<Reference>(m, "Reference")
        // Constructor takes Params
        .def(nb::init<const Params &>())
        .def("get_velocity", [](Reference &ref) { return ref.get_velocity().vector; });
    // Add .def_rw for members like des_vel_x etc. if they are public and need direct access

    // --- Core Module Bindings (Params last in constructor) ---
    nb::class_<FootstepPlanProvider>(m, "FootstepPlanProvider")
        // Constructor: FrameInfo, Reference, State, FootstepPlan&, Params
        .def(nb::init<const FrameInfo &, const Reference &, const State &, FootstepPlan &, const Params &>())
        .def("update", &FootstepPlanProvider::update);

    nb::class_<MovingConstraintProvider>(m, "MovingConstraintProvider")
        // Constructor: FrameInfo, State, FootstepPlan&, Params
        .def(nb::init<const FrameInfo &, const State &, FootstepPlan &, const Params &>())
        .def("update", &MovingConstraintProvider::update);

    nb::class_<ModelPredictiveController>(m, "ModelPredictiveController")
        // Constructor: FrameInfo, State, FootstepPlan&, Params
        .def(nb::init<const FrameInfo &, const State &, FootstepPlan &, const Params &>())
        .def("update", &ModelPredictiveController::update);

    nb::class_<FootTrajectoryGenerator>(m, "FootTrajectoryGenerator")
        // Constructor: FrameInfo, State, FootstepPlan&, Params
        .def(nb::init<const FrameInfo &, const State &, FootstepPlan &, const Params &>())
        .def("update", &FootTrajectoryGenerator::update);

    nb::class_<KalmanFilter>(m, "KalmanFilter")
        // Constructor takes Params
        .def(nb::init<const Params &>())
        .def("update", &KalmanFilter::update);
};

} // namespace python
} // namespace ismpc
