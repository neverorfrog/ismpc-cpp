#include "ismpc_cpp/tools/math/pose2.h"

#include <cmath>
#include <sstream>

namespace ismpc {

Pose2::Pose2(const Vector2& translation) : translation(translation) {}
Pose2::Pose2(const Scalar x, const Scalar y) : translation(x, y) {}
Pose2::Pose2(const Angle rotation) : rotation(rotation) {}
Pose2::Pose2(const Angle rotation, const Vector2& translation) : rotation(rotation), translation(translation) {}
Pose2::Pose2(const Angle rotation, const Scalar x, const Scalar y) : rotation(rotation), translation(x, y) {}

Pose2& Pose2::operator=(const Pose2& other) {
    if (this != &other) {
        rotation = other.rotation;
        translation = other.translation;
    }
    return *this;
}

Pose2& Pose2::operator=(const Vector3& other) {
    rotation = Angle(other(0));
    translation << other(1), other(2);
    return *this;
}

bool Pose2::operator==(const Pose2& other) const {
    return ((translation == other.translation) && (rotation == other.rotation));
}

bool Pose2::operator!=(const Pose2& other) const {
    return !(Pose2(*this) == other);
}

Pose2 Pose2::operator+(const Pose2& other) const {
    return Pose2(*this) += other;
}

Pose2 Pose2::operator-(const Pose2& other) const {
    return Pose2(*this) -= other;
}

Pose2 Pose2::operator-() const {
    return Pose2() - (*this);
}

Vector2 Pose2::operator*(const Vector2& other) const {
    const Scalar s = std::sin(rotation);
    const Scalar c = std::cos(rotation);
    return (Vector2(other.x() * c - other.y() * s, other.x() * s + other.y() * c) + translation);
}

Pose2 Pose2::operator*(const Pose2& other) const {
    return Pose2(*this) *= other;
}

Pose2& Pose2::operator+=(const Pose2& other) {
    translation = *this * other.translation;
    rotation += other.rotation;
    rotation.normalize();
    return *this;
}

Pose2& Pose2::operator-=(const Pose2& other) {
    translation -= other.translation;
    Pose2 p(-other.rotation);
    return *this = p + *this;
}

Pose2& Pose2::operator*=(const Pose2& other) {
    translation = *this * other.translation;
    rotation += other.rotation;
    rotation.normalize();
    return *this;
}

Pose2& Pose2::translate(const Vector2& trans) {
    translation = *this * trans;
    return *this;
}

Pose2& Pose2::translate(const Scalar x, const Scalar y) {
    translation = *this * Vector2(x, y);
    return *this;
}

Pose2& Pose2::rotate(const Angle& rot) {
    rotation += rot;
    return *this;
}

Pose2& Pose2::invert() {
    rotation = -rotation;
    const Vector2 trans = -translation;
    translation = Eigen::Rotation2D<Scalar>(rotation) * trans;
    return *this;
}

Pose2 Pose2::inverse() const {
    return Pose2(*this).invert();
}

Pose2 Pose2::dotMirror() const {
    return Pose2(Angle::normalize(rotation + M_PI), -translation);
}

bool Pose2::isFinite() const {
    return std::isfinite(translation.x()) && std::isfinite(translation.y()) && std::isfinite(rotation);
}

Vector3 Pose2::getVector() const {
    Vector3 vec;
    vec << translation, rotation;
    return vec;
}

std::string Pose2::toString() const {
    std::ostringstream oss;
    oss << this->getVector().transpose().format(Config::CleanFmt);
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Pose2& pose) {
    os << pose.toString();
    return os;
}

}  // namespace ismpc
