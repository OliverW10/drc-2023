#include "../vision.hpp"
#include "../camera.hpp"
#include "../controller.hpp"
#include <gtest/gtest.h>
#include <iostream>

TEST(VisionTests, TestProjection){
    camera::Camera cam{
        camera::getIntrinsics(640, 480),
        Eigen::Matrix4d::Identity(),
        640, 480
    };
    // test center
    Eigen::Vector4d pos{2, 0, 0, 1};
    Eigen::Vector3d actual_screen_pos = camera::projectPoint(cam, pos);
    Eigen::Vector3d expected_screen_pos{320, 240, 1};
    ASSERT_TRUE(actual_screen_pos.isApprox(expected_screen_pos));

}

TEST(VisionTests, TestDistForwardsStraight){
    Eigen::Vector3d result = getDistForwards(0, 1);
    Eigen::Vector3d expected = Eigen::Vector3d(1, 0, 0);
    ASSERT_TRUE(result.isApprox(expected));
}

TEST(VisionTests, TestDistForwardsTurnPos){
    Eigen::Vector3d result = getDistForwards(1, M_PI_2);
    Eigen::Vector3d expected = Eigen::Vector3d(1, 1, M_PI_2);
    ASSERT_TRUE(result.isApprox(expected));
}

TEST(VisionTests, TestDistForwardsStraightRel){
    Eigen::Vector3d result = getDistForwards(0, 2, Eigen::Vector3d(0, 0, 0));
    Eigen::Vector3d expected = Eigen::Vector3d(2, 0, 0);
    ASSERT_TRUE(result.isApprox(expected));
}

TEST(VisionTests, TestDistForwardsTurnAngle){
    Eigen::Vector3d result = getDistForwards(0.5, 1);
    ASSERT_DOUBLE_EQ(result(2), 0.5);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
