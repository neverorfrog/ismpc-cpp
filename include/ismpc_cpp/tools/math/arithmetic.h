#pragma once

#include <cmath>
#include <concepts>
#include <limits>
#include <type_traits>

#include "ismpc_cpp/types/math_types.h"

namespace ismpc {
namespace Arithmetic {

template <typename T>
concept Number = std::is_arithmetic_v<T> && std::totally_ordered<T>;

template <Number T>
constexpr T truncateToDecimalPlaces(const T& number, const int decimalPlaces) {
    Scalar factor = std::pow(10.0, decimalPlaces);
    return std::trunc(number * factor) / factor;
}

/**
 * @brief Sign function
 */
template <Number T>
constexpr int sgn(const T& number) {
    return (T(0) < number) - (number < T(0));
}

/**
 * @brief Sign function (0 is considered positive)
 */
template <Number T>
constexpr T sgnPos(const T& number) {
    return (number >= T(0)) - (number < T(0));
}

/**
 * @brief Sign function (0 is considered negative)
 */
template <Number T>
constexpr T sgnNeg(const T& number) {
    return (number > T(0)) - (number <= T(0));
}

template <Number T>
bool isZero(T number, Scalar eps = std::numeric_limits<Scalar>::epsilon()) {
    return std::abs(number) < eps;
}

template <Number T>
bool isEqual(T a, T b, Scalar eps = std::numeric_limits<Scalar>::epsilon()) {
    const T diff = std::abs(a - b);
    return diff < eps || diff < eps * std::max(std::abs(a), std::abs(b));
}

template <Number T>
T cubic(T t) {
    return -2 * t * t * t + 3 * t * t;
}

template <Number T>
T cubic_dot(T t) {
    return -6 * t * t + 6 * t;
}

template <Number T>
T cubic_ddot(T t) {
    return -12 * t + 6;
}

template <Number T>
T quartic(T t) {
    return 16  * std::pow(t, 4) - 32 * std::pow(t, 3) + 16 * std::pow(t, 2);
}

template <Number T>
T quartic_dot(T t) {
    return 64 * std::pow(t, 3) - 96  * std::pow(t, 2) + 32 * t;
}

template <Number T>
T quartic_ddot(T t) {
    return 192 * std::pow(t, 2) - 192 * t + 32;
}

}  // namespace Arithmetic
}  // namespace ismpc
