#include "ismpc_cpp/tools/config/config.h"

#include <gtest/gtest.h>

#include "ismpc_cpp/tools/config/robot_config.h"

namespace ismpc {

TEST(RobotConfig, Equality) {
    EXPECT_NEAR(RobotConfig::h, 0.78, 0.001);
}

}  // namespace ismpc
