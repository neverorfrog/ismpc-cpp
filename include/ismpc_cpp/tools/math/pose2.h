#pragma once

#include <Eigen/Geometry>
#include <string>

#include "angle.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

struct Pose2 {
    Angle rotation{0};
    Vector2 translation{0, 0};

    Pose2() = default;
    Pose2(const Pose2& other) = default;
    explicit Pose2(const Vector2& translation);
    Pose2(const Scalar x, const Scalar y);
    explicit Pose2(const Angle rotation);
    Pose2(const Angle rotation, const Vector2& translation);
    Pose2(const Angle rotation, const Scalar x, const Scalar y);

    Pose2& operator=(const Pose2& other);
    Pose2& operator=(const Vector3& other);

    bool operator==(const Pose2& other) const;
    bool operator!=(const Pose2& other) const;

    Pose2 operator+(const Pose2& other) const;
    Pose2 operator-(const Pose2& other) const;
    Pose2 operator-() const;
    Vector2 operator*(const Vector2& other) const;
    Pose2 operator*(const Pose2& other) const;

    Pose2& operator+=(const Pose2& other);
    Pose2& operator-=(const Pose2& other);
    Pose2& operator*=(const Pose2& other);

    Pose2& translate(const Vector2& trans);
    Pose2& translate(const Scalar x, const Scalar y);
    Pose2& rotate(const Angle& rot);
    Pose2& invert();
    Pose2 inverse() const;

    Pose2 dotMirror() const;
    bool isFinite() const;
    Vector3 getVector() const;
    std::string toString() const;

    friend std::ostream& operator<<(std::ostream& os, const Pose2& pose);
};

}  // namespace ismpc
