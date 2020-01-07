#include <gtest/gtest.h>
#include "algorithms/Kalman1D.h"


TEST(kalman_d1_test, init) {

    AutoDrive::Algorithms::Kalman1D kalman(0.1, 0.5);
    EXPECT_EQ(kalman.getPosition(), 0);
    EXPECT_EQ(kalman.getSpeed(), 0);
}


TEST(kalman_d1_test, iteration) {

    AutoDrive::Algorithms::Kalman1D kalman{0.1, 0.5};
    double measured_pose = 0;
    double dt = 0.1;
    for (int i = 0 ; i < 20 ; i++) {
        if(i*dt >= 1) {
            measured_pose = 1.0;
        }
        std::cout << "Iteration " << i+1 << std::endl;
        std::cout << "Prediciton, states: " << kalman.getPosition() << " " << kalman.getSpeed() << std::endl;
        kalman.predict(dt, 0);
        cv::Mat measurement = (cv::Mat_<double>(2, 1) <<
                                                      measured_pose,  // p
                0);
        std::cout << "Measurement, states: " << kalman.getPosition() << " " << kalman.getSpeed() << std::endl;
        kalman.correct(measurement);
    }
}


TEST(kalman_d1_test, fixed_pose) {

    AutoDrive::Algorithms::Kalman1D kalman{0.1, 0.5};
    double measured_pose = 10;
    double dt = 0.1;
    for (int i = 0 ; i < 1000 ; i++) {
        kalman.predict(dt, 0);
        cv::Mat measurement = (cv::Mat_<double>(2, 1) <<
                measured_pose,
                0);
        kalman.correct(measurement);
    }

    EXPECT_NEAR(kalman.getPosition(), measured_pose, 0.1);
    EXPECT_NEAR(kalman.getSpeed(), 0, 0.1);
}


TEST(kalman_d1_test, fixed_speed) {

    AutoDrive::Algorithms::Kalman1D kalman{0.1, 0.5};
    double dt = 0.1;
    double speed = 1;
    double measured_pose = 0;
    for (int i = 0 ; i < 1000 ; i++) {

        measured_pose = dt * i * speed;
        kalman.predict(dt, 0);
        cv::Mat measurement = (cv::Mat_<double>(2, 1) <<
                measured_pose,
                speed);
        kalman.correct(measurement);
    }

    EXPECT_NEAR(kalman.getPosition(), measured_pose, 0.1);
    EXPECT_NEAR(kalman.getSpeed(), speed, 0.1);
}

int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


