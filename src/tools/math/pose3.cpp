#include "ismpc_cpp/tools/math/pose3.h"

namespace ismpc {

Pose3::Pose3(const Vector3& translation) : translation(translation) {}

Pose3::Pose3(const Scalar x, const Scalar y, const Scalar z) : translation(x, y, z) {}

Pose3::Pose3(const RotationMatrix& rotation) : rotation(rotation) {}

Pose3::Pose3(const RotationMatrix& rotation, const Vector3& translation)
    : rotation(rotation), translation(translation) {}

Pose3::Pose3(const RotationMatrix& rotation, const Scalar x, const Scalar y, const Scalar z)
    : rotation(rotation), translation(x, y, z) {}

Pose3::Pose3(const Pose2& pose2) {
    translation << pose2.translation(0), pose2.translation(1), 0;
    rotation = RotationMatrix::aroundZ(pose2.rotation);
}

Pose3& Pose3::operator=(const Pose3& other) {
    rotation = other.rotation;
    translation = other.translation;
    return *this;
}

bool Pose3::operator==(const Pose3& other) const {
    return ((translation == other.translation) && (rotation == other.rotation));
}

bool Pose3::operator!=(const Pose3& other) const {
    return !(*this == other);
}

Pose3 Pose3::operator+(const Pose3& other) const {
    return Pose3(*this) += other;
}

Pose3 Pose3::operator+(const Vector3& trans) const {
    return Pose3(rotation, *this * trans);
}

Vector3 Pose3::operator*(const Vector3& other) const {
    return (rotation * other) + translation;
}

Pose3 Pose3::operator*(const Pose3& other) const {
    return Pose3(rotation * other.rotation, *this * other.translation);
}

Pose3 Pose3::operator*(const RotationMatrix& rot) const {
    return Pose3(rotation * rot, translation);
}

Pose3& Pose3::operator+=(const Pose3& other) {
    translation = *this * other.translation;
    rotation += other.rotation;
    rotation.normalize();
    return *this;
}

Pose3& Pose3::operator+=(const Vector3& trans) {
    translation = *this * trans;
    return *this;
}

Pose3& Pose3::operator*=(const Pose3& other) {
    translation = *this * other.translation;
    rotation *= other.rotation;
    return *this;
}

Pose3& Pose3::operator*=(const RotationMatrix& rot) {
    rotation *= rot;
    return *this;
}

Pose3 Pose3::operator-(const Pose3& other) const {
    Vector3 translation = this->translation - other.translation;
    RotationMatrix rotation = this->rotation.inverse() * other.rotation;
    return Pose3(rotation, translation);
}

Pose3& Pose3::conc(const Pose3& other) {
    return *this *= other;
}

Pose3& Pose3::translate(const Vector3& trans) {
    return *this += trans;
}

Pose3& Pose3::translate(Scalar x, Scalar y, Scalar z) {
    return *this += Vector3(x, y, z);
}

Pose3 Pose3::translated(const Vector3& trans) const {
    return *this + trans;
}

Pose3 Pose3::translated(const Scalar x, const Scalar y, const Scalar z) const {
    return *this + Vector3(x, y, z);
}

Pose3& Pose3::rotate(const RotationMatrix& rot) {
    return *this *= rot;
}

Pose3& Pose3::rotateX(Scalar angle) {
    return *this *= RotationMatrix::aroundX(angle);
}

Pose3& Pose3::rotateY(Scalar angle) {
    return *this *= RotationMatrix::aroundY(angle);
}

Pose3& Pose3::rotateZ(Scalar angle) {
    return *this *= RotationMatrix::aroundZ(angle);
}

Pose3& Pose3::invert() {
    rotation = rotation.inverse();
    translation = rotation * -translation;
    return *this;
}

Pose3 Pose3::inverse() const {
    return Pose3(*this).invert();
}

Matrix4 Pose3::getHomogen() const {
    Matrix4 m = Eigen::Matrix4d::Zero();
    m.block<3, 3>(0, 0) = rotation;
    m.block<3, 1>(0, 3) = translation;
    return m;
}

Pose2 Pose3::getPose2() const {
    Pose2 pose{};
    pose.translation << translation(0), translation(1);
    pose.rotation = rotation.getZAngle();
    return pose;
}

Vector6 Pose3::getVector() const {
    Vector6 pose_vector{};
    pose_vector.segment(0, 3) = translation;
    pose_vector.segment(3, 3) = rotation.getRPY();
    return pose_vector;
}

Pose3 Pose3::relativeTo(const Pose3& other) const {
    Matrix4 homogen = getHomogen().inverse() * other.getHomogen();
    RotationMatrix rotation = RotationMatrix(homogen.block<3, 3>(0, 0));
    return Pose3(rotation, homogen.block<3, 1>(0, 3));
}

std::string Pose3::toString() const {
    std::ostringstream oss;
    oss << "Translation: " << translation.transpose() << std::endl;
    oss << "Rotation: \n" << rotation << std::endl;
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Pose3& pose) {
    os << pose.toString();
    return os;
}

}  // namespace ismpc
