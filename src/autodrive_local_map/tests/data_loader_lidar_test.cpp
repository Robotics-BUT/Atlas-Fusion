#include "gtest/gtest.h"
#include <iostream>
#include <istream>

#include "data_loader/LidarDataLoader.h"

#define DATA_FOLDER TEST_FOLDER"test_data/"

TEST(data_loader_lidar_test, data_loader_lidar_data)
{
    AutoDrive::DataLoader::LidarDataLoader dataLoader(AutoDrive::DataLoader::LidarIdentifier::kLeftLidar);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),500);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::LidarScanDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kLidarScanDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745501937363);
    EXPECT_EQ(data->getInnerTimestamp(), 6735310785341193029);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::LidarScanDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kLidarScanDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745602125461);
    EXPECT_EQ(data->getInnerTimestamp(), 6735310785341293114);
}

TEST(data_loader_lidar_test, load_lidar_get_timestamp) {

    AutoDrive::DataLoader::LidarDataLoader dataLoader(AutoDrive::DataLoader::LidarIdentifier::kLeftLidar);
    dataLoader.loadData(DATA_FOLDER);

    EXPECT_EQ(dataLoader.isOnEnd(), false);
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745501937363);
    auto data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745602125461);
    data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745702684022);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    while(!dataLoader.isOnEnd()) {
    data = dataLoader.getNextData();
    }
    EXPECT_EQ(dataLoader.isOnEnd(), true);
}


TEST(data_loader_lidar_test, load_lidar_set_pose) {

    AutoDrive::DataLoader::LidarDataLoader dataLoader(AutoDrive::DataLoader::LidarIdentifier::kLeftLidar);
    dataLoader.loadData(DATA_FOLDER);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    dataLoader.setPose(1568186755544302267);
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186755544302268);
    auto data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186755644891576);
    data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186755744975909);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    while(!dataLoader.isOnEnd()) {
        data = dataLoader.getNextData();
    }
    EXPECT_EQ(dataLoader.isOnEnd(), true);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}