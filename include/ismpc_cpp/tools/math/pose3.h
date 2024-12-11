#pragma once

#include <sstream>
#include <string>

#include "ismpc_cpp/tools/math/pose2.h"
#include "ismpc_cpp/tools/math/rotation_matrix.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

/**
 * @brief Represents a 3D pose consisting of a rotation and a translation.
 *
 * The Pose3 struct encapsulates a 3D pose, which includes a rotation matrix and a translation vector.
 * It provides various constructors for initializing the pose and a wide range of operators and methods
 * for manipulating and transforming the pose.
 */
struct Pose3 {
    RotationMatrix rotation{};
    Vector3 translation{0, 0, 0};

    Pose3() = default;
    Pose3(const Pose3& other) = default;

    explicit Pose3(const Vector3& translation);
    Pose3(const Scalar x, const Scalar y, const Scalar z);
    explicit Pose3(const RotationMatrix& rotation);
    Pose3(const RotationMatrix& rotation, const Vector3& translation);
    Pose3(const RotationMatrix& rotation, const Scalar x, const Scalar y, const Scalar z);
    explicit Pose3(const Pose2& pose2);

    /**
     * @brief Equality operator to compare two Pose3 objects.
     *
     * This operator checks if the current Pose3 object is equal to another Pose3 object.
     *
     * @param other The Pose3 object to compare with.
     * @return true if both Pose3 objects are equal, false otherwise.
     */
    bool operator==(const Pose3& other) const;

    /**
     * @brief Inequality operator to compare two Pose3 objects.
     *
     * This operator checks if the current Pose3 object is not equal to another Pose3 object.
     *
     * @param other The Pose3 object to compare with.
     * @return true if both Pose3 objects are not equal, false otherwise.
     */
    bool operator!=(const Pose3& other) const;

    /**
     * @brief Addition operator to add two Pose3 objects.
     *
     * This operator adds the current Pose3 object to another Pose3 object.
     *
     * @param other The Pose3 object to add.
     * @return The resulting Pose3 object after addition.
     */
    Pose3 operator+(const Pose3& other) const;
    Pose3& operator+=(const Pose3& other);

    /**
     * @brief Addition operator to add a translation vector to the Pose3 object.
     *
     * This operator adds a translation vector to the current Pose3 object.
     *
     */
    Pose3 operator+(const Vector3& trans) const;
    Pose3& operator+=(const Vector3& trans);

    /**
     * @brief Multiplication operator to transform a vector by the Pose3 object.
     *
     * This operator transforms a vector by the current Pose3 object.
     *
     * @param other The vector to transform.
     * @return The resulting vector after transformation.
     */
    Vector3 operator*(const Vector3& other) const;

    /**
     * @brief Multiplication operator to concatenate two Pose3 objects.
     *
     * This operator concatenates the current Pose3 object with another Pose3 object.
     *
     * @param other The Pose3 object to concatenate.
     * @return The resulting Pose3 object after concatenation.
     */
    Pose3 operator*(const Pose3& other) const;
    Pose3& operator*=(const Pose3& other);

    /**
     * @brief Multiplication operator to transform the Pose3 object by a rotation matrix.
     *
     * This operator transforms the current Pose3 object by a rotation matrix.
     *
     * @param rot The rotation matrix to transform the Pose3 object.
     * @return The resulting Pose3 object after transformation.
     */
    Pose3 operator*(const RotationMatrix& rot) const;
    Pose3& operator*=(const RotationMatrix& rot);

    /**
     * @brief Concatenation operator to concatenate two Pose3 objects.
     *
     * This operator concatenates the current Pose3 object with another Pose3 object.
     *
     * @param other The Pose3 object to concatenate.
     * @return The resulting Pose3 object after concatenation.
     */
    Pose3& conc(const Pose3& other);

    /**
     * @brief Translate the Pose3 object by a translation vector.
     *
     * This method translates the current Pose3 object by a translation vector
     */
    Pose3& translate(const Vector3& trans);

    /**
     * @brief Translate the Pose3 object by a translation vector.
     *
     * This method translates the current Pose3 object by a translation vector
     */
    Pose3& translate(Scalar x, Scalar y, Scalar z);

    /**
     * @brief Translate the Pose3 object by a translation vector.
     *
     * This method translates the current Pose3 object by a translation vector and
     * returns a new Pose3 object with the translation applied.
     */
    Pose3 translated(const Vector3& trans) const;

    /**
     * @brief Translate the Pose3 object by a translation vector.
     *
     * This method translates the current Pose3 object by a translation vector and
     * returns a new Pose3 object with the translation applied.
     */
    Pose3 translated(const Scalar x, const Scalar y, const Scalar z) const;
    Pose3& rotate(const RotationMatrix& rot);
    Pose3& rotateX(Scalar angle);
    Pose3& rotateY(Scalar angle);
    Pose3& rotateZ(Scalar angle);
    Pose3& invert();
    Pose3 inverse() const;
    Matrix4 getHomogen() const;
    Pose2 getPose2() const;
    Vector6 getVector() const;
    Pose3 relativeTo(const Pose3& other) const;
    std::string toString() const;

    friend std::ostream& operator<<(std::ostream& os, const Pose3& pose);
};

}  // namespace ismpc
