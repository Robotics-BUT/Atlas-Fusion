#pragma once


namespace AutoDrive {
    namespace DataLoader {

        enum class DataModelTypes {
            kCameraDataModelType,
            kCameraIrDataModelType,
            kGnssPositionDataModelType,
            kGnssTimeDataModelType,
            kImuDquatDataModelType,
            kImuGnssDataModelType,
            kImuImuDataModelType,
            kImuMagDataModelType,
            kImuPressDataModelType,
            kImuTempDataModelType,
            kImuTimeDataModelType,
            kLidarScanDataModelType,
            kYoloDetectionDataModelType,
            kGenericDataModelType,
            kErrorDataModelType,
        };

        enum class LidarIdentifier {
            kLeftLidar,
            kRightLidar,
        };

        enum class CameraIndentifier {
            kCameraLeftFront,
            kCameraLeftSide,
            kCameraRightFront,
            kCameraRightSide,
            kCameraIr,
        };
    }
}