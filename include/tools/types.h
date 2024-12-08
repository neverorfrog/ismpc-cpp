#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

using Scalar = double;
using Matrix = Eigen::Matrix<Scalar, -1, -1>;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
using VectorX = Eigen::VectorX<Scalar>;
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using Quaternion = Eigen::Quaternion<Scalar>;
using AngleAxis = Eigen::AngleAxis<Scalar>;