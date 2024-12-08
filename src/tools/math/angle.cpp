#include "tools/math/angle.h"

namespace ismpc {

constexpr Angle::Angle(Scalar angle) : value(angle) {}

Angle::operator Scalar&() {
  return value;
}

constexpr Angle::operator const Scalar&() const {
  return value;
}

constexpr Angle Angle::operator-() const {
  return Angle(-value);
}

Angle& Angle::operator+=(Scalar angle) {
  value += angle;
  return *this;
}

Angle& Angle::operator-=(Scalar angle) {
  value -= angle;
  return *this;
}

Angle& Angle::operator*=(Scalar angle) {
  value *= angle;
  return *this;
}

Angle& Angle::operator/=(Scalar angle) {
  value /= angle;
  return *this;
}

}  // namespace ismpc