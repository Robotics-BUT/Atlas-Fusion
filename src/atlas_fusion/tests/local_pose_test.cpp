/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <gtest/gtest.h>
#include "data_models/local_map/LocalPosition.h"


TEST(local_pose_test, init) {

    AtlasFusion::DataModels::LocalPosition pose{rtl::Vector3D<double>{0.0, 0.0, 0.0}, rtl::Quaternion<double>::identity(), 0};

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

    AtlasFusion::DataModels::LocalPosition pose{rtl::Vector3D<double>{0.0, 0.0, 0.0}, rtl::Quaternion<double>::identity(), 0};
    AtlasFusion::DataModels::LocalPosition poseDiff{{1, 0, 0}, {0.707, 0, 0, 0.707}, 1};
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

    AtlasFusion::DataModels::LocalPosition pose{rtl::Vector3D<double>{0.0, 0.0, 0.0}, rtl::Quaternion<double>::identity(), 0};
    AtlasFusion::DataModels::LocalPosition endPose{{1, 0, 0}, {0.707, 0, 0, 0.707}, 1};
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

    AtlasFusion::DataModels::LocalPosition pose{{1.1, 3.2, 6.7}, {0.458, -0.222, -0.430, 0.746}, 50};
    AtlasFusion::DataModels::LocalPosition endPose{{4.5, 6.7, 2.7}, {0.783, 0.330, -0.508, 0.141}, 150};
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


