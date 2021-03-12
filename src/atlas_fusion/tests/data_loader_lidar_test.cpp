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

#include "gtest/gtest.h"
#include <iostream>
#include <istream>

#include "data_loader/LidarDataLoader.h"
#include "Context.h"

#define DATA_FOLDER TEST_FOLDER"test_data/"

TEST(data_loader_lidar_test, data_loader_lidar_data)
{
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::DataLoader::LidarDataLoader dataLoader(context, AtlasFusion::DataLoader::LidarIdentifier::kLeftLidar);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),500);

    auto data = std::dynamic_pointer_cast<AtlasFusion::DataModels::LidarScanDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AtlasFusion::DataModels::DataModelTypes::kLidarScanDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745501937363);
    EXPECT_EQ(data->getInnerTimestamp(), 6735310785341193029);

    data = std::dynamic_pointer_cast<AtlasFusion::DataModels::LidarScanDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AtlasFusion::DataModels::DataModelTypes::kLidarScanDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745602125461);
    EXPECT_EQ(data->getInnerTimestamp(), 6735310785341293114);
}

TEST(data_loader_lidar_test, load_lidar_get_timestamp) {

    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::DataLoader::LidarDataLoader dataLoader(context, AtlasFusion::DataLoader::LidarIdentifier::kLeftLidar);
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

    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::DataLoader::LidarDataLoader dataLoader(context, AtlasFusion::DataLoader::LidarIdentifier::kLeftLidar);
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