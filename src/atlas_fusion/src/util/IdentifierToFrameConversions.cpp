//
// Created by standa on 21.12.22.
//

#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive {

    std::string frameTypeFromDataModel(const std::shared_ptr <DataModels::GenericDataModel> &dataModel) {
        auto type = dataModel->getType();
        switch (type) {
            case DataModels::DataModelTypes::kCameraDataModelType:
                switch (std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(dataModel)->getCameraIdentifier()) {
                    case DataLoader::CameraIndentifier::kCameraLeftFront:
                        return LocalMap::Frames::kCameraLeftFront;
                    case DataLoader::CameraIndentifier::kCameraLeftSide:
                        return LocalMap::Frames::kCameraLeftSide;
                    case DataLoader::CameraIndentifier::kCameraRightFront:
                        return LocalMap::Frames::kCameraRightFront;
                    case DataLoader::CameraIndentifier::kCameraRightSide:
                        return LocalMap::Frames::kCameraRightSide;
                    default:
                        throw std::runtime_error("Unable to convert between camera dataModel and frame type!");
                }

            case DataModels::DataModelTypes::kCameraIrDataModelType:
                return LocalMap::Frames::kCameraIr;

            case DataModels::DataModelTypes::kLidarScanDataModelType:
                switch (std::dynamic_pointer_cast<DataModels::LidarScanDataModel>(dataModel)->getLidarIdentifier()) {
                    case DataLoader::LidarIdentifier::kLeftLidar:
                        return LocalMap::Frames::kLidarLeft;
                    case DataLoader::LidarIdentifier::kRightLidar:
                        return LocalMap::Frames::kLidarRight;
                    case DataLoader::LidarIdentifier::kCenterLidar:
                        return LocalMap::Frames::kLidarCenter;
                    default:
                        throw std::runtime_error("Unable to convert between lidar dataModel and frame type!");
                }
            case DataModels::DataModelTypes::kRadarTiScanDataModelType:
                return LocalMap::Frames::kRadarTi;

            case DataModels::DataModelTypes::kImuDquatDataModelType:
            case DataModels::DataModelTypes::kImuGnssDataModelType:
            case DataModels::DataModelTypes::kImuImuDataModelType:
            case DataModels::DataModelTypes::kImuMagDataModelType:
            case DataModels::DataModelTypes::kImuPressDataModelType:
            case DataModels::DataModelTypes::kImuTimeDataModelType:
            case DataModels::DataModelTypes::kImuTempDataModelType:
                return LocalMap::Frames::kImuFrame;

            case DataModels::DataModelTypes::kGnssPositionDataModelType:
            case DataModels::DataModelTypes::kGnssTimeDataModelType:
                return LocalMap::Frames::kGnssAntennaRear;

            default:
                throw std::runtime_error("Unable to convert between sensor dataModel and frame type!");
        }
    }

    std::string frameTypeFromIdentifier(const DataLoader::CameraIndentifier &identifier) {
        switch (identifier) {
            case DataLoader::CameraIndentifier::kCameraLeftFront:
                return LocalMap::Frames::kCameraLeftFront;
            case DataLoader::CameraIndentifier::kCameraLeftSide:
                return LocalMap::Frames::kCameraLeftSide;
            case DataLoader::CameraIndentifier::kCameraRightFront:
                return LocalMap::Frames::kCameraRightFront;
            case DataLoader::CameraIndentifier::kCameraRightSide:
                return LocalMap::Frames::kCameraRightSide;
            case DataLoader::CameraIndentifier::kCameraIr:
                return LocalMap::Frames::kCameraIr;
            default:
                throw std::runtime_error("Unable to convert between camera identifier and frame type!");
        }
    }
}