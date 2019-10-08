#include <iostream>

#include "gtest/gtest.h"

#include "algorithms/GenericAlgorithm.h"
#include "local_map/GenericLocalMap.h"
#include "visualizers/GenericVisializer.h"

TEST(generic_test, algorithm_test)
{
    AutoDrive::Algorithms::GenericAlgorithm alg;
    alg.doCoolStaff();
}

TEST(generic_test, local_map_test)
{
    AutoDrive::LocalMap::GenericLocalMap map;
    map.doMapping();
}

TEST(generic_test, visualizer_test)
{
    AutoDrive::Visualizers::GenericVisualizer visualizer;
    visualizer.visualizeSomething();
}


TEST(generic_test, simple_test)
{
    EXPECT_EQ(1, 1);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}