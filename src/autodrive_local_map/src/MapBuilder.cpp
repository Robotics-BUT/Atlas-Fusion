#include <chrono>
#include "MapBuilder.h"
#include "data_loader/data_models/all.h"
#include "data_loader/data_models/DataModelTypes.h"

namespace AutoDrive {


    void MapBuilder::loadData(const std::string& dataFolder) {
        visualizationHandler_.drawTestingCube();

        dataLoader_.loadData(dataFolder);
        std::cout << "Total No. of loaded data: " << dataLoader_.getDataSize() << std::endl;

    }

    void MapBuilder::buildMap() {

        int64_t last_system_ts = 0;
        uint64_t last_data_ts = 0;

        for (size_t i = 0; i < dataLoader_.getDataSize(); i++) {
            auto data = dataLoader_.getNextData();
            auto data_ts = data->getTimestamp();
            std::cout << data_ts << " " << data->toString() << std::endl;

            if(last_system_ts == 0) {
                last_system_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                last_data_ts = data_ts;
            }

            while(true) {
                auto system_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                if((data_ts - last_data_ts) < (system_ts - last_system_ts)*maxReplayerRate_) {
                    last_data_ts = data_ts;
                    last_system_ts = system_ts;
                    break;
                }
            }


            auto dataType = data->getType();


            /* ... data processing ... */

            if(dataType == DataLoader::DataModelTypes::kCameraDataModelType) {

                auto cameraFrame = std::dynamic_pointer_cast<DataLoader::CameraFrameDataModel>(data);
                visualizationHandler_.drawRGBImage(cameraFrame);

            } else if (dataType == DataLoader::DataModelTypes::kCameraIrDataModelType) {

                auto irCameraFrame = std::dynamic_pointer_cast<DataLoader::CameraIrFrameDataModel>(data);
                visualizationHandler_.drawIRImage(irCameraFrame);

            } else if (dataType == DataLoader::DataModelTypes::kGnssPositionDataModelType) {

                auto poseData = std::dynamic_pointer_cast<DataLoader::GnssPoseDataModel>(data);
                visualizationHandler_.drawGnssPoseData(poseData);

            } else if (dataType == DataLoader::DataModelTypes::kGnssTimeDataModelType) {

            } else if (dataType == DataLoader::DataModelTypes::kImuDquatDataModelType) {

            } else if (dataType == DataLoader::DataModelTypes::kImuGnssDataModelType) {

            } else if (dataType == DataLoader::DataModelTypes::kImuImuDataModelType) {

                auto imuData = std::dynamic_pointer_cast<DataLoader::ImuImuDataModel>(data);
                visualizationHandler_.drawImuData(imuData);

            } else if (dataType == DataLoader::DataModelTypes::kImuMagDataModelType) {

            } else if (dataType == DataLoader::DataModelTypes::kImuPressDataModelType) {

            } else if (dataType == DataLoader::DataModelTypes::kImuTempDataModelType) {

            } else if (dataType == DataLoader::DataModelTypes::kImuTimeDataModelType) {

            } else if (dataType == DataLoader::DataModelTypes::kLidarScanDataModelType) {

                auto lidarData = std::dynamic_pointer_cast<DataLoader::LidarScanDataModel>(data);
                visualizationHandler_.drawLidarData(lidarData);

            } else if (dataType == DataLoader::DataModelTypes::kGenericDataModelType) {
                logger_.warning("Received Generic data model from DataLoader");
            } else if (dataType == DataLoader::DataModelTypes::kErrorDataModelType) {
                logger_.warning("Received Error data model from DataLoader");
            } else {
                logger_.warning("Unepected type of data model from DataLoader");
            }
        }
    }


    void MapBuilder::clearData() {
        dataLoader_.clear();
    }
}