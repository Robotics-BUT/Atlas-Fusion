#include "gtest/gtest.h"
#include <iostream>
#include <istream>

#include "data_loader/DataLoader.h"
#include "data_loader/CameraDataLoader.h"
#include "data_loader/LidarDataLoader.h"
#include "data_loader/GnssDataLoader.h"
#include "data_loader/ImuDataLoader.h"

#define DATA_FOLDER TEST_FOLDER"test_data/"
#define TEST_ERR_TOLERANCE 1e-16

TEST(data_loader_imu_test, load_imu_dquat)
{
    AutoDrive::DataLoader::ImuDataLoader dataLoader(AutoDrive::DataLoader::ImuLoaderIdentifier::kDQuat);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),499);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuDquatDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuDquatDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745408734467);
    EXPECT_NEAR(data->getDQuat().x(), -2.43261e-06, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getDQuat().y(), 8.03731e-06, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getDQuat().z(), 2.96906e-06, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getDQuat().w(), 1, TEST_ERR_TOLERANCE);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuDquatDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuDquatDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745411234467);
    EXPECT_NEAR(data->getDQuat().x(), -4.37163e-06, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getDQuat().y(), 2.85357e-06, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getDQuat().z(), 2.42144e-07, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getDQuat().w(), 1, TEST_ERR_TOLERANCE);
}

TEST(data_loader_imu_test, load_imu_gnss)
{
    AutoDrive::DataLoader::ImuDataLoader dataLoader(AutoDrive::DataLoader::ImuLoaderIdentifier::kGnss);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),499);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuGnssDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuGnssDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745425234180);
    EXPECT_NEAR(data->getLatitude(), 49.22807312, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getLongitude(), 16.57549667, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAltitude(), 0.0, TEST_ERR_TOLERANCE);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuGnssDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuGnssDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745445234180);
    EXPECT_NEAR(data->getLatitude(), 49.22807312, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getLongitude(), 16.57549667, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAltitude(), 0.0, TEST_ERR_TOLERANCE);
}

TEST(data_loader_imu_test, load_imu_imu)
{
    AutoDrive::DataLoader::ImuDataLoader dataLoader(AutoDrive::DataLoader::ImuLoaderIdentifier::kImu);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),499);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuImuDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuImuDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745408734467);
    EXPECT_NEAR(data->getLinearAcc().x(), 0.346637, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getLinearAcc().y(), -0.395, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getLinearAcc().z(), -9.55036, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAngularVel().x(), -0.00194611, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAngularVel().y(), 0.00642984, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAngularVel().z(), 0.00237526, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getOrientation().x(), 0.016559, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getOrientation().y(), 0.0288746, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getOrientation().z(), 0.0474863, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getOrientation().w(), 0.998317, TEST_ERR_TOLERANCE);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuImuDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuImuDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745411234467);
    EXPECT_NEAR(data->getLinearAcc().x(), 0.389194, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getLinearAcc().y(), -0.415689, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getLinearAcc().z(), -9.60509, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAngularVel().x(), -0.0034973, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAngularVel().y(), 0.00228286, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getAngularVel().z(), 0.000193725, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getOrientation().x(), 0.016556, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getOrientation().y(), 0.0288707, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getOrientation().z(), 0.0474868, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getOrientation().w(), 0.998317, TEST_ERR_TOLERANCE);
}

TEST(data_loader_imu_test, load_imu_mag)
{
    AutoDrive::DataLoader::ImuDataLoader dataLoader(AutoDrive::DataLoader::ImuLoaderIdentifier::kMag);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),499);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuMagDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuMagDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745410202085);
    EXPECT_NEAR(data->getMagneticField().x(), 0.240066, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getMagneticField().y(), 0.943281, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getMagneticField().z(), -1.00343, TEST_ERR_TOLERANCE);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuMagDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuMagDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745420202085);
    EXPECT_NEAR(data->getMagneticField().x(), 0.239775, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getMagneticField().y(), 0.946174, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getMagneticField().z(), -0.996268, TEST_ERR_TOLERANCE);
}

TEST(data_loader_imu_test, load_imu_press)
{
    AutoDrive::DataLoader::ImuDataLoader dataLoader(AutoDrive::DataLoader::ImuLoaderIdentifier::kPressure);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),499);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuPressureDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuPressDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745410202085);
    EXPECT_EQ(data->getPressure(), 98879);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuPressureDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuPressDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745430202085);
    EXPECT_EQ(data->getPressure(), 98885);
}

TEST(data_loader_imu_test, load_imu_temp)
{
    AutoDrive::DataLoader::ImuDataLoader dataLoader(AutoDrive::DataLoader::ImuLoaderIdentifier::kTemp);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),499);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuTempDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuTempDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745407734180);
    EXPECT_EQ(data->getTemperature(), 29.0625);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuTempDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuTempDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745410234180);
    EXPECT_EQ(data->getTemperature(), 29.0625);
}

TEST(data_loader_imu_test, load_imu_time)
{
    AutoDrive::DataLoader::ImuDataLoader dataLoader(AutoDrive::DataLoader::ImuLoaderIdentifier::kTime);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),499);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuTimeDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuTimeDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745407734180);
    EXPECT_EQ(data->getYear(), 2019);
    EXPECT_EQ(data->getMonth(), 9);
    EXPECT_EQ(data->getDay(), 11);
    EXPECT_EQ(data->getHour(), 7);
    EXPECT_EQ(data->getMinute(), 25);
    EXPECT_EQ(data->getSec(), 45);
    EXPECT_EQ(data->getNSec(), 403600000);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::ImuTimeDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kImuTimeDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745410234180);
    EXPECT_EQ(data->getYear(), 2019);
    EXPECT_EQ(data->getMonth(), 9);
    EXPECT_EQ(data->getDay(), 11);
    EXPECT_EQ(data->getHour(), 7);
    EXPECT_EQ(data->getMinute(), 25);
    EXPECT_EQ(data->getSec(), 45);
    EXPECT_EQ(data->getNSec(), 406100000);
}

TEST(data_loader_imu_test, load_imu_get_timestamp) {

    AutoDrive::DataLoader::ImuDataLoader dataLoader(AutoDrive::DataLoader::ImuLoaderIdentifier::kDQuat);
    dataLoader.loadData(DATA_FOLDER);

    EXPECT_EQ(dataLoader.isOnEnd(), false);
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745408734467);
    auto data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745411234467);
    data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745413734467);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    while(!dataLoader.isOnEnd()) {
        data = dataLoader.getNextData();
    }
    EXPECT_EQ(dataLoader.isOnEnd(), true);
}


TEST(data_loader_imu_test, load_imu_set_pose) {

    AutoDrive::DataLoader::ImuDataLoader dataLoader(AutoDrive::DataLoader::ImuLoaderIdentifier::kDQuat);
    dataLoader.loadData(DATA_FOLDER);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    dataLoader.setPose(1568186745911234466);
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745911234467);
    auto data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745913734467);
    data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745916234467);

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