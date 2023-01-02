//
// Created by standa on 21.12.22.
//

#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive {

    FrameType frameTypeFromDataModel(const std::shared_ptr <DataModels::GenericDataModel> &dataModel) {
        auto type = dataModel->getType();
        switch (type) {
            case DataModels::DataModelTypes::kCameraDataModelType:
                switch (std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(dataModel)->getCameraIdentifier()) {
                    case DataLoader::CameraIndentifier::kCameraLeftFront:
                        return FrameType::kCameraLeftFront;
                    case DataLoader::CameraIndentifier::kCameraLeftSide:
                        return FrameType::kCameraLeftSide;
                    case DataLoader::CameraIndentifier::kCameraRightFront:
                        return FrameType::kCameraRightFront;
                    case DataLoader::CameraIndentifier::kCameraRightSide:
                        return FrameType::kCameraRightSide;
                    default:
                        throw std::runtime_error("Unable to convert between camera dataModel and frame type!");
                }

            case DataModels::DataModelTypes::kCameraIrDataModelType:
                return FrameType::kCameraIr;

            case DataModels::DataModelTypes::kLidarScanDataModelType:
                switch (std::dynamic_pointer_cast<DataModels::LidarScanDataModel>(dataModel)->getLidarIdentifier()) {
                    case DataLoader::LidarIdentifier::kLeftLidar:
                        return FrameType::kLidarLeft;
                    case DataLoader::LidarIdentifier::kRightLidar:
                        return FrameType::kLidarRight;
                    case DataLoader::LidarIdentifier::kCenterLidar:
                        return FrameType::kLidarCenter;
                    default:
                        throw std::runtime_error("Unable to convert between lidar dataModel and frame type!");
                }
            case DataModels::DataModelTypes::kRadarTiScanDataModelType:
                return FrameType::kRadarTi;

            case DataModels::DataModelTypes::kImuDquatDataModelType:
            case DataModels::DataModelTypes::kImuGnssDataModelType:
            case DataModels::DataModelTypes::kImuImuDataModelType:
            case DataModels::DataModelTypes::kImuMagDataModelType:
            case DataModels::DataModelTypes::kImuPressDataModelType:
            case DataModels::DataModelTypes::kImuTimeDataModelType:
            case DataModels::DataModelTypes::kImuTempDataModelType:
                return FrameType::kImu;

            case DataModels::DataModelTypes::kGnssPositionDataModelType:
            case DataModels::DataModelTypes::kGnssTimeDataModelType:
                return FrameType::kGnssAntennaRear;

            default:
                throw std::runtime_error("Unable to convert between sensor dataModel and frame type!");
        }
    }

    FrameType frameTypeFromIdentifier(const DataLoader::CameraIndentifier &identifier) {
        switch (identifier) {
            case DataLoader::CameraIndentifier::kCameraLeftFront:
                return FrameType::kCameraLeftFront;
            case DataLoader::CameraIndentifier::kCameraLeftSide:
                return FrameType::kCameraLeftSide;
            case DataLoader::CameraIndentifier::kCameraRightFront:
                return FrameType::kCameraRightFront;
            case DataLoader::CameraIndentifier::kCameraRightSide:
                return FrameType::kCameraRightSide;
            case DataLoader::CameraIndentifier::kCameraIr:
                return FrameType::kCameraIr;
            default:
                throw std::runtime_error("Unable to convert between camera identifier and frame type!");
        }
    }

    std::string frameTypeName(const FrameType& frame) {
        switch(frame) {
            case FrameType::kOrigin: return "origin";
            case FrameType::kLidarLeft: return "lidar_left";
            case FrameType::kLidarRight: return "lidar_right";
            case FrameType::kLidarCenter: return "lidar_center";
            case FrameType::kRadarTi: return "radar_ti";
            case FrameType::kImu: return "imu";
            case FrameType::kCameraLeftFront: return "camera_left_front";
            case FrameType::kCameraLeftSide: return "camera_left_side";
            case FrameType::kCameraRightFront: return "camera_right_front";
            case FrameType::kCameraRightSide: return "camera_right_side";
            case FrameType::kCameraIr: return "camera_ir";
            case FrameType::kGnssAntennaFront: return "gnss_front";
            case FrameType::kGnssAntennaRear: return "gnss_rear";
            default: throw std::runtime_error("Unknown frame type name!");
        }
    }
}