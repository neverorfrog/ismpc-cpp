#pragma once

#include <vector>

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
  torso,
  neck,
  head,

  // Right Arm
  rShoulder,
  rBiceps,
  rElbow,
  rForearm,
  rHand,

  // Left Arm
  lShoulder,
  lBiceps,
  lElbow,
  lForearm,
  lHand,

  // Right Leg
  rPelvis,
  rHip,
  rThigh,
  rTibia,
  rAnkle,
  rFoot,

  // Left Leg
  lPelvis,
  lHip,
  lThigh,
  lTibia,
  lAnkle,
  lFoot,

  numOfLimbs
};

enum class FsrSensor : int { fr, fl, br, bl };

// Enum for the joints of the Nao robot in the same order as in the URDF
enum class Joint : int {
  // Head
  headYaw,
  headPitch,

  // Left Leg
  lHipYawPitch,
  lHipRoll,
  lHipPitch,
  lKneePitch,
  lAnklePitch,
  lAnkleRoll,

  // Left Arm
  lShoulderPitch,
  lShoulderRoll,
  lElbowYaw,
  lElbowRoll,
  lWristYaw,
  lHand,

  // Right Leg
  rHipYawPitch,  // this joint is virtual but BHuman Kinematics uses it
  rHipRoll,
  rHipPitch,
  rKneePitch,
  rAnklePitch,
  rAnkleRoll,

  // Right Arm
  rShoulderPitch,
  rShoulderRoll,
  rElbowYaw,
  rElbowRoll,
  rWristYaw,
  rHand,

  numOfJoints,
};

inline std::string toString(Joint joint) {
  switch (joint) {
    case Joint::headYaw:
      return "headYaw";
    case Joint::headPitch:
      return "headPitch";
    case Joint::lShoulderPitch:
      return "lShoulderPitch";
    case Joint::lShoulderRoll:
      return "lShoulderRoll";
    case Joint::lElbowYaw:
      return "lElbowYaw";
    case Joint::lElbowRoll:
      return "lElbowRoll";
    case Joint::lWristYaw:
      return "lWristYaw";
    case Joint::lHipYawPitch:
      return "lHipYawPitch";
    case Joint::lHipRoll:
      return "lHipRoll";
    case Joint::lHipPitch:
      return "lHipPitch";
    case Joint::lKneePitch:
      return "lKneePitch";
    case Joint::lAnklePitch:
      return "lAnklePitch";
    case Joint::lAnkleRoll:
      return "lAnkleRoll";
    case Joint::rHipRoll:
      return "rHipRoll";
    case Joint::rHipPitch:
      return "rHipPitch";
    case Joint::rKneePitch:
      return "rKneePitch";
    case Joint::rAnklePitch:
      return "rAnklePitch";
    case Joint::rAnkleRoll:
      return "rAnkleRoll";
    case Joint::rShoulderPitch:
      return "rShoulderPitch";
    case Joint::rShoulderRoll:
      return "rShoulderRoll";
    case Joint::rElbowYaw:
      return "rElbowYaw";
    case Joint::rElbowRoll:
      return "rElbowRoll";
    case Joint::rWristYaw:
      return "rWristYaw";
    case Joint::lHand:
      return "lHand";
    case Joint::rHand:
      return "rHand";
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
