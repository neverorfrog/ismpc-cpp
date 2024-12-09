#pragma once

#include <cmath>
#include <string>

#include "tools/math/arithmetic.h"
#include "types/math_types.h"

namespace ismpc {

/**
 * Representation for 3x3 RotationMatrices
 */
class RotationMatrix : public Matrix3 {
   public:
    RotationMatrix();
    RotationMatrix(const Matrix3& other);
    RotationMatrix(const AngleAxis& angleAxis);
    RotationMatrix(const Quaternion& quat);

    RotationMatrix& operator=(const Matrix3& other);
    RotationMatrix& operator=(const AngleAxis& angleAxis);
    RotationMatrix& operator=(const Quaternion& quat);

    Vector3 operator*(const Vector3& vector) const;
    RotationMatrix operator*(const RotationMatrix& other) const;
    RotationMatrix& operator*=(const RotationMatrix& rot);
    RotationMatrix& operator*=(const AngleAxis& rot);
    RotationMatrix& operator*=(const Quaternion& rot);

    RotationMatrix& invert();
    RotationMatrix inverse() const;

    RotationMatrix& rotateX(const Scalar angle);
    RotationMatrix& rotateY(const Scalar angle);
    RotationMatrix& rotateZ(const Scalar angle);

    Scalar getXAngle() const;
    Scalar getYAngle() const;
    Scalar getZAngle() const;

    Vector3 getRPY() const;

    static RotationMatrix aroundX(const Scalar angle);
    static RotationMatrix aroundY(const Scalar angle);
    static RotationMatrix aroundZ(const Scalar angle);
};

}  // namespace ismpc
