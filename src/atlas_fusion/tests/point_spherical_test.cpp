#include <gtest/gtest.h>
#include "algorithms/pointcloud/SphericalPoint.h"

#define allowed_error 1e-4

TEST(projector_test, initialization) {

    auto point1 = AtlasFusion::Algorithms::SphericalPoint(0.0, 0.0, 0.0);
    EXPECT_EQ(point1.radius(), 0.0);
    EXPECT_EQ(point1.theta(), 0.0);
    EXPECT_EQ(point1.phi(), 0.0);

    auto point2 = AtlasFusion::Algorithms::SphericalPoint(1.0, 2.0, 3.0);
    EXPECT_EQ(point2.radius(), 1.0);
    EXPECT_EQ(point2.theta(), 2.0);
    EXPECT_EQ(point2.phi(), 3.0);
}

TEST(projector_test, fromXYZ) {

    auto point1 = AtlasFusion::Algorithms::SphericalPoint<float>::fromXYZ(1.0, 0.0, 0.0);
    EXPECT_NEAR(point1.radius(), 1.0, allowed_error);
    EXPECT_NEAR(point1.theta(), M_PI_2, allowed_error);
    EXPECT_NEAR(point1.phi(), 0.0, allowed_error);

    auto back_conversion = point1.toXYT();
    EXPECT_NEAR(back_conversion.x(), 1.0, allowed_error);
    EXPECT_NEAR(back_conversion.y(), 0.0, allowed_error);
    EXPECT_NEAR(back_conversion.z(), 0.0, allowed_error);


    for (float x = 0.1 ; x < 5 ; x += 0.1) {
        for (float y = 0.1 ; y < 5 ; y += 0.1) {
            for (float z = -5 ; z < 5 ; z += 0.1) {
                auto point2 = AtlasFusion::Algorithms::SphericalPoint<float>::fromXYZ(x, y, z);
                auto back_conversion_2 = point2.toXYT();
                EXPECT_NEAR(back_conversion_2.x(), x, allowed_error);
                EXPECT_NEAR(back_conversion_2.y(), y, allowed_error);
                EXPECT_NEAR(back_conversion_2.z(), z, allowed_error);
            }
        }
    }
}



int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}