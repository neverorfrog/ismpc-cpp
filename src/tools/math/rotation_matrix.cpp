#include "tools/math/rotation_matrix.h"

namespace ismpc {

RotationMatrix::RotationMatrix() : Matrix3(Matrix3::Identity()) {}

RotationMatrix::RotationMatrix(const Matrix3& other) : Matrix3(other) {}

RotationMatrix::RotationMatrix(const AngleAxis& angleAxis) : Matrix3(angleAxis.toRotationMatrix()) {}

RotationMatrix::RotationMatrix(const Quaternion& quat) : Matrix3(quat.toRotationMatrix()) {}

RotationMatrix& RotationMatrix::operator=(const Matrix3& other) {
    Matrix3::operator=(other);
    return *this;
}

RotationMatrix& RotationMatrix::operator=(const AngleAxis& angleAxis) {
    Matrix3::operator=(angleAxis.toRotationMatrix());
    return *this;
}

RotationMatrix& RotationMatrix::operator=(const Quaternion& quat) {
    Matrix3::operator=(quat.toRotationMatrix());
    return *this;
}

Vector3 RotationMatrix::operator*(const Vector3& vector) const {
    return Matrix3::operator*(vector);
}

RotationMatrix RotationMatrix::operator*(const RotationMatrix& other) const {
    return RotationMatrix(Base::operator*(other));
}

RotationMatrix& RotationMatrix::operator*=(const RotationMatrix& rot) {
    Matrix3::operator*=(rot);
    return *this;
}

RotationMatrix& RotationMatrix::operator*=(const AngleAxis& rot) {
    Matrix3::operator*=(rot.toRotationMatrix());
    return *this;
}

RotationMatrix& RotationMatrix::operator*=(const Quaternion& rot) {
    Matrix3::operator*=(rot.toRotationMatrix());
    return *this;
}

RotationMatrix& RotationMatrix::invert() {
    transposeInPlace();
    return *this;
}

RotationMatrix RotationMatrix::inverse() const {
    return RotationMatrix(transpose());
}

RotationMatrix& RotationMatrix::rotateX(const Scalar angle) {
    Matrix3& mat = *this;
    const Scalar c = std::cos(angle);
    const Scalar s = std::sin(angle);
    const Scalar m01 = mat(0, 1);
    const Scalar m11 = mat(1, 1);
    const Scalar m21 = mat(2, 1);
    mat(0, 1) = m01 * c + mat(0, 2) * s;
    mat(1, 1) = m11 * c + mat(1, 2) * s;
    mat(2, 1) = m21 * c + mat(2, 2) * s;
    mat(0, 2) = mat(0, 2) * c - m01 * s;
    mat(1, 2) = mat(1, 2) * c - m11 * s;
    mat(2, 2) = mat(2, 2) * c - m21 * s;
    return *this;
}

RotationMatrix& RotationMatrix::rotateY(const Scalar angle) {
    Matrix3& mat = *this;
    const Scalar c = std::cos(angle);
    const Scalar s = std::sin(angle);
    const Scalar m00 = mat(0, 0);
    const Scalar m10 = mat(1, 0);
    const Scalar m20 = mat(2, 0);
    mat(0, 0) = m00 * c - mat(0, 2) * s;
    mat(1, 0) = m10 * c - mat(1, 2) * s;
    mat(2, 0) = m20 * c - mat(2, 2) * s;
    mat(0, 2) = mat(0, 2) * c + m00 * s;
    mat(1, 2) = mat(1, 2) * c + m10 * s;
    mat(2, 2) = mat(2, 2) * c + m20 * s;
    return *this;
}

RotationMatrix& RotationMatrix::rotateZ(const Scalar angle) {
    Matrix3& mat = *this;
    const Scalar c = std::cos(angle);
    const Scalar s = std::sin(angle);
    const Scalar m00 = mat(0, 0);
    const Scalar m10 = mat(1, 0);
    const Scalar m20 = mat(2, 0);
    mat(0, 0) = m00 * c + mat(0, 1) * s;
    mat(1, 0) = m10 * c + mat(1, 1) * s;
    mat(2, 0) = m20 * c + mat(2, 1) * s;
    mat(0, 1) = mat(0, 1) * c - m00 * s;
    mat(1, 1) = mat(1, 1) * c - m10 * s;
    mat(2, 1) = mat(2, 1) * c - m20 * s;
    return *this;
}

Scalar RotationMatrix::getXAngle() const {
    const Matrix3& mat = *this;
    const Scalar h = std::sqrt(mat(1, 2) * mat(1, 2) + mat(2, 2) * mat(2, 2));
    if (Arithmetic::isZero(h))
        return 0.0;
    else
        return std::acos(mat(2, 2) / h) * -Arithmetic::sgnNeg(mat(1, 2));
}

Scalar RotationMatrix::getYAngle() const {
    const Matrix3& mat = *this;
    const float h = std::sqrt(mat(0, 0) * mat(0, 0) + mat(2, 0) * mat(2, 0));
    if (Arithmetic::isZero(h))
        return 0.f;
    else
        return std::acos(mat(0, 0) / h) * -Arithmetic::sgnNeg(mat(2, 0));
}

Scalar RotationMatrix::getZAngle() const {
    const Matrix3& mat = *this;
    const float h = std::sqrt(mat(0, 0) * mat(0, 0) + mat(1, 0) * mat(1, 0));
    if (Arithmetic::isZero(h))
        return 0.f;
    else
        return std::acos(mat(0, 0) / h) * Arithmetic::sgnPos(mat(1, 0));
}

Vector3 RotationMatrix::getRPY() const {
    const Matrix3& mat = *this;
    Vector3 result{};
    result << std::atan2(mat(2, 1), mat(2, 2)),
        atan2(-mat(2, 0), std::sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2))),
        std::atan2(mat(1, 0), mat(0, 0));
    return result;
}

RotationMatrix RotationMatrix::aroundX(const Scalar angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return (RotationMatrix() << 1.f, 0.f, 0.f, 0.f, c, -s, 0.f, s, c).finished();
}

RotationMatrix RotationMatrix::aroundY(const Scalar angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return (RotationMatrix() << c, 0.f, s, 0.f, 1.f, 0.f, -s, 0.f, c).finished();
}

RotationMatrix RotationMatrix::aroundZ(const Scalar angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return (RotationMatrix() << c, -s, 0.f, s, c, 0.f, 0.f, 0.f, 1.f).finished();
}

}  // namespace ismpc
