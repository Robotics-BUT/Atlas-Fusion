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

#include "Context.h"
#include "data_loader/DataLoader.h"
#include "data_loader/GnssDataLoader.h"

#define DATA_FOLDER TEST_FOLDER"test_data/"
#define TEST_ERR_TOLERANCE 1e-16

TEST(data_loader_gnss_test, load_gnss_pose)
{
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::DataLoader::GnssDataLoader dataLoader(context, AtlasFusion::DataLoader::GnssLoaderIdentifier::kPose);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),500);

    auto data = std::dynamic_pointer_cast<AtlasFusion::DataModels::GnssPoseDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AtlasFusion::DataModels::DataModelTypes::kGnssPositionDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745436549395);
    EXPECT_NEAR(data->getLatitude(), 49.22810527, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getLongitude(), 16.57547959, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAltitude(), 281.368, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAzimut(), 63.5158, TEST_ERR_TOLERANCE);

    data = std::dynamic_pointer_cast<AtlasFusion::DataModels::GnssPoseDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AtlasFusion::DataModels::DataModelTypes::kGnssPositionDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745472765184);
    EXPECT_NEAR(data->getLatitude(), 49.22810527, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getLongitude(), 16.5754796, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAltitude(), 281.373, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAzimut(), 63.5235, TEST_ERR_TOLERANCE);
}

TEST(data_loader_gnss_test, load_gnss_time)
{
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::DataLoader::GnssDataLoader dataLoader(context, AtlasFusion::DataLoader::GnssLoaderIdentifier::kTime);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),500);

    auto data = std::dynamic_pointer_cast<AtlasFusion::DataModels::GnssTimeDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AtlasFusion::DataModels::DataModelTypes::kGnssTimeDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745436549395);
    EXPECT_EQ(data->getYear(), 2019);
    EXPECT_EQ(data->getMonth(), 9);
    EXPECT_EQ(data->getDay(), 11);
    EXPECT_EQ(data->getHour(), 7);
    EXPECT_EQ(data->getMinute(), 25);
    EXPECT_EQ(data->getSec(), 45);
    EXPECT_EQ(data->getNSec(), 430000000);

    data = std::dynamic_pointer_cast<AtlasFusion::DataModels::GnssTimeDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AtlasFusion::DataModels::DataModelTypes::kGnssTimeDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745472765184);
    EXPECT_EQ(data->getYear(), 2019);
    EXPECT_EQ(data->getMonth(), 9);
    EXPECT_EQ(data->getDay(), 11);
    EXPECT_EQ(data->getHour(), 7);
    EXPECT_EQ(data->getMinute(), 25);
    EXPECT_EQ(data->getSec(), 45);
    EXPECT_EQ(data->getNSec(), 470000000);
}


TEST(data_loader_gnss_test, load_imu_get_timestamp) {

    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::DataLoader::GnssDataLoader dataLoader(context, AtlasFusion::DataLoader::GnssLoaderIdentifier::kPose);
    dataLoader.loadData(DATA_FOLDER);

    EXPECT_EQ(dataLoader.isOnEnd(), false);
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745436549395);
    auto data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745472765184);
    data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745522796400);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    while(!dataLoader.isOnEnd()) {
        data = dataLoader.getNextData();
    }
    EXPECT_EQ(dataLoader.isOnEnd(), true);
}


TEST(data_loader_gnss_test, load_gnss_set_pose) {

    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::DataLoader::GnssDataLoader dataLoader(context, AtlasFusion::DataLoader::GnssLoaderIdentifier::kPose);
    dataLoader.loadData(DATA_FOLDER);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    dataLoader.setPose(1568186758510724030);
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186758510724031);
    auto data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186758561084750);
    data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186758622647153);

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