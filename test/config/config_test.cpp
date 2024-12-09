#include "tools/config/config.h"

#include <gtest/gtest.h>

#include "tools/config/robot_config.h"

namespace ismpc {

TEST(Config, Equality) {
    EXPECT_EQ(Config::W, 100);
}

TEST(RobotConfig, Equality) {
    EXPECT_NEAR(RobotConfig::h, 0.78, 0.001);
}

}  // namespace ismpc
