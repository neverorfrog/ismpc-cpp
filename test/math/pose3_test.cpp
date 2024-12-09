#include "tools/math/pose3.h"

#include <gtest/gtest.h>

namespace ismpc {

TEST(Pose3, Equality) {
    Pose3 pose = Pose3(Vector3(0, 0, 0));
    Pose3 pose2 = Pose3(Vector3(1, 1, 1));
    Vector3 translation = Vector3(1, 1, 1);
    EXPECT_EQ(pose + translation, pose2);
}

}  // namespace ismpc
