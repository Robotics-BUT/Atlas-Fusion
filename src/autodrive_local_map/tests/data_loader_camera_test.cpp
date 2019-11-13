#include "gtest/gtest.h"
#include <iostream>
#include <istream>

#include "data_loader/DataLoader.h"
#include "data_loader/CameraDataLoader.h"

#define DATA_FOLDER TEST_FOLDER"test_data/"
#define TEST_ERR_TOLERANCE 1e-16

TEST(data_loader_camera_test, load_camera_rgb)
{
    AutoDrive::DataLoader::CameraDataLoader dataLoader(AutoDrive::DataLoader::CameraIndentifier::kCameraLeftFront);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),500);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraFrameDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745453388439);
    auto frame = data->getImage();
    EXPECT_EQ(frame.rows, 1200);
    EXPECT_EQ(frame.cols, 1920);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraFrameDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745553488239);
    frame = data->getImage();
    EXPECT_EQ(frame.rows, 1200);
    EXPECT_EQ(frame.cols, 1920);
}


TEST(data_loader_camera_test, load_ir_camera_ir)
{
    AutoDrive::DataLoader::CameraDataLoader dataLoader(AutoDrive::DataLoader::CameraIndentifier::kCameraIr);
    dataLoader.loadData(DATA_FOLDER);
    EXPECT_EQ(dataLoader.getDataSize(),500);

    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraIrFrameDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraIrDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745426675402);
    EXPECT_NEAR(data->getTemp().first, -29.65, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getTemp().second, 20.74, TEST_ERR_TOLERANCE);
    auto frame = data->getImage();
    EXPECT_EQ(frame.rows, 512);
    EXPECT_EQ(frame.cols, 640);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraIrFrameDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraIrDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745463015536);
    EXPECT_NEAR(data->getTemp().first, -29.65, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(data->getTemp().second, 20.74, TEST_ERR_TOLERANCE);
    frame = data->getImage();
    EXPECT_EQ(frame.rows, 512);
    EXPECT_EQ(frame.cols, 640);
}


TEST(data_loader_camera_test, load_camera_get_timestamp) {

    AutoDrive::DataLoader::CameraDataLoader dataLoader(AutoDrive::DataLoader::CameraIndentifier::kCameraLeftFront);
    dataLoader.loadData(DATA_FOLDER);

    EXPECT_EQ(dataLoader.isOnEnd(), false);
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745453388439);
    auto data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745553488239);
    data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186745653588039);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    while(!dataLoader.isOnEnd()) {
        data = dataLoader.getNextData();
    }
    EXPECT_EQ(dataLoader.isOnEnd(), true);
}


TEST(data_loader_camera_test, load_camera_set_pose) {

    AutoDrive::DataLoader::CameraDataLoader dataLoader(AutoDrive::DataLoader::CameraIndentifier::kCameraLeftFront);
    dataLoader.loadData(DATA_FOLDER);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    dataLoader.setPose(1568186758466362438);
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186758466362439);
    auto data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186758566462239);
    data = dataLoader.getNextData();
    EXPECT_EQ(dataLoader.getLowestTimestamp(), 1568186758666562039);

    EXPECT_EQ(dataLoader.isOnEnd(), false);

    while(!dataLoader.isOnEnd()) {
        data = dataLoader.getNextData();
    }
    EXPECT_EQ(dataLoader.isOnEnd(), true);
}



TEST(data_loader_camera_test, load_camera_yolo_detections) {

    AutoDrive::DataLoader::CameraDataLoader dataLoader(AutoDrive::DataLoader::CameraIndentifier::kCameraLeftFront);
    dataLoader.loadData(DATA_FOLDER);


    auto data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraFrameDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745453388439);
    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraFrameDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745553488239);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraFrameDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745653588039);
    auto yoloDet = data->getYoloDetections();
    EXPECT_EQ(yoloDet.size(), 1);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().x1_, 0);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().y1_, 779);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().x2_, 70);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().y2_, 1128);
    EXPECT_NEAR(yoloDet.at(0)->getDetectionConfidence(), 0.8603562712669373, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(yoloDet.at(0)->getClassConfidence(), 0.9875117540359497, TEST_ERR_TOLERANCE);
    EXPECT_EQ(yoloDet.at(0)->getReducedDetectionClass(), AutoDrive::DataLoader::ReducedYoloDetectionClasses::kVehicle);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraFrameDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745753687839);
    yoloDet = data->getYoloDetections();
    EXPECT_EQ(yoloDet.size(), 1);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().x1_, 17);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().y1_, 665);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().x2_, 465);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().y2_, 1232);
    EXPECT_NEAR(yoloDet.at(0)->getDetectionConfidence(), 0.9980521202087402, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(yoloDet.at(0)->getClassConfidence(), 0.9979549646377563, TEST_ERR_TOLERANCE);
    EXPECT_EQ(yoloDet.at(0)->getReducedDetectionClass(), AutoDrive::DataLoader::ReducedYoloDetectionClasses::kVehicle);

    data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraFrameDataModel>(dataLoader.getNextData());
    EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraDataModelType);
    EXPECT_EQ(data->getTimestamp(), 1568186745853787639);
    yoloDet = data->getYoloDetections();
    EXPECT_EQ(yoloDet.size(), 1);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().x1_, 14);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().y1_, 596);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().x2_, 836);
    EXPECT_EQ(yoloDet.at(0)->getBoundingBox().y2_, 1206);
    EXPECT_NEAR(yoloDet.at(0)->getDetectionConfidence(), 0.9997450709342957, TEST_ERR_TOLERANCE);
    EXPECT_NEAR(yoloDet.at(0)->getClassConfidence(), 0.9995705485343933, TEST_ERR_TOLERANCE);
    EXPECT_EQ(yoloDet.at(0)->getReducedDetectionClass(), AutoDrive::DataLoader::ReducedYoloDetectionClasses::kVehicle);

    while(!dataLoader.isOnEnd()) {
        data = std::dynamic_pointer_cast<AutoDrive::DataLoader::CameraFrameDataModel>(dataLoader.getNextData());
        EXPECT_EQ(data->getType(), AutoDrive::DataLoader::DataModelTypes::kCameraDataModelType);

        yoloDet = data->getYoloDetections();
        if(yoloDet.size() > 1) {
            EXPECT_EQ(data->getTimestamp(), 1568186751259176839); // first time there are two detection at frame no. 58
            return;
        }
    }

    EXPECT_ANY_THROW("Error when loading multiple yolo detections for single frame");
}



int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}