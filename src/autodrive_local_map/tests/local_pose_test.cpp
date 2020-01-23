#include <gtest/gtest.h>
#include "data_models/local_map/LocalPosition.h"


TEST(local_pose_test, init) {

    AutoDrive::DataModels::LocalPosition pose{{}, {}, 0};

    EXPECT_EQ(pose.getPosition().x(), 0);
    EXPECT_EQ(pose.getPosition().y(), 0);
    EXPECT_EQ(pose.getPosition().z(), 0);

    EXPECT_EQ(pose.getOrientation().x(), 0);
    EXPECT_EQ(pose.getOrientation().y(), 0);
    EXPECT_EQ(pose.getOrientation().z(), 0);
    EXPECT_EQ(pose.getOrientation().w(), 1);

    EXPECT_EQ(pose.getTimestamp(), 0);
}

TEST(local_pose_test, adding) {

    AutoDrive::DataModels::LocalPosition pose{{}, {}, 0};
    AutoDrive::DataModels::LocalPosition poseDiff{{1,0,0}, {0.707, 0, 0, 0.707}, 1};
    auto endPose = pose + poseDiff;

    EXPECT_EQ(endPose.getPosition().x(), 1);
    EXPECT_EQ(endPose.getPosition().y(), 0);
    EXPECT_EQ(endPose.getPosition().z(), 0);

    EXPECT_EQ(endPose.getOrientation().x(), 0);
    EXPECT_EQ(endPose.getOrientation().y(), 0);
    EXPECT_EQ(endPose.getOrientation().z(), 0.707);
    EXPECT_EQ(endPose.getOrientation().w(), 0.707);

    EXPECT_EQ(endPose.getTimestamp(), 1);

    endPose = pose + poseDiff + poseDiff;

    EXPECT_EQ(endPose.getPosition().x(), 2);
    EXPECT_EQ(endPose.getPosition().y(), 0);
    EXPECT_EQ(endPose.getPosition().z(), 0);

    EXPECT_NEAR(endPose.getOrientation().x(), 0, 0.001);
    EXPECT_NEAR(endPose.getOrientation().y(), 0, 0.001);
    EXPECT_NEAR(endPose.getOrientation().z(), 1, 0.001);
    EXPECT_NEAR(endPose.getOrientation().w(), 0, 0.001);

    EXPECT_EQ(endPose.getTimestamp(), 2);
}

TEST(local_pose_test, substracting) {

    AutoDrive::DataModels::LocalPosition pose{{}, {}, 0};
    AutoDrive::DataModels::LocalPosition endPose{{1,0,0}, {0.707, 0, 0, 0.707}, 1};
    auto poseDiff = endPose - pose;

    EXPECT_EQ(poseDiff.getPosition().x(), 1);
    EXPECT_EQ(poseDiff.getPosition().y(), 0);
    EXPECT_EQ(poseDiff.getPosition().z(), 0);

    EXPECT_EQ(poseDiff.getOrientation().x(), 0);
    EXPECT_EQ(poseDiff.getOrientation().y(), 0);
    EXPECT_NEAR(poseDiff.getOrientation().z(), 0.707, 0.001);
    EXPECT_NEAR(poseDiff.getOrientation().w(), 0.707, 0.001);

    EXPECT_EQ(poseDiff.getTimestamp(), 1);
}


TEST(local_pose_test, diff_cycle) {

    AutoDrive::DataModels::LocalPosition pose{{1.1, 3.2, 6.7}, {0.458, -0.222, -0.430, 0.746}, 50};
    AutoDrive::DataModels::LocalPosition endPose{{4.5,6.7,2.7}, {0.783, 0.330, -0.508, 0.141}, 150};
    auto poseDiff = endPose - pose;
    auto newPose = pose + poseDiff;

    EXPECT_NEAR(endPose.getPosition().x(), newPose.getPosition().x(), 0.001);
    EXPECT_NEAR(endPose.getPosition().y(), newPose.getPosition().y(), 0.001);
    EXPECT_NEAR(endPose.getPosition().z(), newPose.getPosition().z(), 0.001);
    EXPECT_NEAR(endPose.getOrientation().x(), newPose.getOrientation().x(), 0.001);
    EXPECT_NEAR(endPose.getOrientation().y(), newPose.getOrientation().y(), 0.001);
    EXPECT_NEAR(endPose.getOrientation().z(), newPose.getOrientation().z(), 0.001);
    EXPECT_NEAR(endPose.getOrientation().w(), newPose.getOrientation().w(), 0.001);
    EXPECT_EQ(endPose.getTimestamp(), newPose.getTimestamp());

}

int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


