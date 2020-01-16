#include <gtest/gtest.h>
#include <cmath>
#include "algorithms/QuadratureFilter.h"

TEST(quadrature_filter_test, init) {

    AutoDrive::Algorithms::QuadratureFilter filter;
}

TEST(quadrature_filter_test, basic_filtration) {

    AutoDrive::Algorithms::QuadratureFilter filter;
    filter.setState(0);

    for(int i = 0 ; i < 720 ; i += 10) {
        double angle = i * M_PI / 180;

        filter.prediction( angle-filter.getState() );
        filter.measurement( angle , 0.5 );
        std::cout << "angle: " << filter.getState() << " " << angle << std::endl;
    }
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}