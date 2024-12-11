#include "ismpc_cpp/tools/math/arithmetic.h"

#include <gtest/gtest.h>

using namespace ismpc::Arithmetic;

namespace ismpc {

TEST(Sign, Equality) {
    EXPECT_EQ(sgn(1), 1);
}

TEST(Sign, Equality2) {
    EXPECT_EQ(sgn(-1), -1);
}

TEST(Sign, Equality3) {
    EXPECT_EQ(sgn(0), 0);
}

TEST(Truncate, Equality) {
    EXPECT_EQ(truncateToDecimalPlaces(1.23456789, 2), 1.23);
}

TEST(isZero, Equality) {
    EXPECT_TRUE(isZero(0));
}

TEST(isZero, Equality2) {
    EXPECT_FALSE(isZero(0.1));
}

TEST(isZero, Equality3) {
    EXPECT_TRUE(isZero(0.00000000000000000000000000000000000001));
}

TEST(isEqual, Equality) {
    EXPECT_TRUE(isEqual(0.1, 0.1));
}

}  // namespace ismpc
