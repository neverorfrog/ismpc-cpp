#pragma once

#include <stdexcept>
#include <string>

namespace ismpc {

enum class Arm : int { left, right, numOfArms };

enum class Leg : int { left, right, numOfLegs };

enum class Foot : int { left, right, numOfFeet };

inline std::string toString(Foot foot) {
    switch (foot) {
        case Foot::left:
            return "LEFT";
        case Foot::right:
            return "RIGHT";
        default:
            return "UNKNOWN";
    }
}

inline std::ostream& operator<<(std::ostream& os, Foot foot) {
    os << toString(foot);
    return os;
}

enum class Limb : int {
    Torso,
    Neck,
    Head,

    // Right Arm
    RShoulder,
    RBiceps,
    RElbow,
    RForearm,
    RHand,

    // Left Arm
    LShoulder,
    LBiceps,
    LElbow,
    LForearm,
    LHand,

    // Right Leg
    RPelvis,
    RHip,
    RThigh,
    RTibia,
    RAnkle,
    RFoot,

    // Left Leg
    LPelvis,
    LHip,
    LThigh,
    LTibia,
    LAnkle,
    LFoot,

    numOfLimbs
};

enum class FsrSensor : int { fr, fl, br, bl };

// Enum for the joints of the Nao robot in the same order as in the URDF
enum class Joint : int {
    // Head
    HeadYaw,
    HeadPitch,

    // Left Leg
    LHipYawPitch,
    LHipRoll,
    LHipPitch,
    LKneePitch,
    LAnklePitch,
    LAnkleRoll,

    // Left Arm
    LShoulderPitch,
    LShoulderRoll,
    LElbowYaw,
    LElbowRoll,
    LWristYaw,
    LHand,

    // Right Leg
    RHipYawPitch,  // this joint is virtual (not present in Lola) but present in the URDF
    RHipRoll,
    RHipPitch,
    RKneePitch,
    RAnklePitch,
    RAnkleRoll,

    // Right Arm
    RShoulderPitch,
    RShoulderRoll,
    RElbowYaw,
    RElbowRoll,
    RWristYaw,
    RHand,

    numOfJoints,
};

inline std::string toString(Joint joint) {
    switch (joint) {
        case Joint::HeadYaw:
            return "HeadYaw";
        case Joint::HeadPitch:
            return "HeadPitch";
        case Joint::LShoulderPitch:
            return "LShoulderPitch";
        case Joint::LShoulderRoll:
            return "LShoulderRoll";
        case Joint::LElbowYaw:
            return "LElbowYaw";
        case Joint::LElbowRoll:
            return "LElbowRoll";
        case Joint::LWristYaw:
            return "LWristYaw";
        case Joint::LHipYawPitch:
            return "LHipYawPitch";
        case Joint::LHipRoll:
            return "LHipRoll";
        case Joint::LHipPitch:
            return "LHipPitch";
        case Joint::LKneePitch:
            return "LKneePitch";
        case Joint::LAnklePitch:
            return "LAnklePitch";
        case Joint::LAnkleRoll:
            return "LAnkleRoll";
        case Joint::RHipRoll:
            return "RHipRoll";
        case Joint::RHipPitch:
            return "RHipPitch";
        case Joint::RKneePitch:
            return "RKneePitch";
        case Joint::RAnklePitch:
            return "RAnklePitch";
        case Joint::RAnkleRoll:
            return "RAnkleRoll";
        case Joint::RShoulderPitch:
            return "RShoulderPitch";
        case Joint::RShoulderRoll:
            return "RShoulderRoll";
        case Joint::RElbowYaw:
            return "RElbowYaw";
        case Joint::RElbowRoll:
            return "RElbowRoll";
        case Joint::RWristYaw:
            return "RWristYaw";
        case Joint::LHand:
            return "LHand";
        case Joint::RHand:
            return "RHand";
        default:
            return "UNKNOWN (" + std::to_string(static_cast<int>(joint)) + ")";
            throw std::invalid_argument("Unknown Joint value");
    }
}

inline std::ostream& operator<<(std::ostream& os, Joint joint) {
    os << toString(joint);
    return os;
}

template <typename Enum>
constexpr auto operator+(Enum e) noexcept {
    return static_cast<std::underlying_type_t<Enum>>(e);
}

}  // namespace ismpc
