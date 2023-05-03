#include "../vision.cpp"
#include <gtest/gtest.h>
#include <iostream>

TEST(VisionTests, TestProjection){
    Camera cam{
        getIntrinsics(),
        Eigen::Matrix4d::Identity(),
        640, 480
    };
    // test center
    Eigen::Vector4d pos{2, 0, 0, 1};
    Eigen::Vector3d actual_screen_pos = projectPoint(cam, pos);
    Eigen::Vector3d expected_screen_pos{320, 240, 1};
    ASSERT_EQ(actual_screen_pos, expected_screen_pos);

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}