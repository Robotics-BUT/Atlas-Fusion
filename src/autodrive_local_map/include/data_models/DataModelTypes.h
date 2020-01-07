#pragma once


namespace AutoDrive::DataModels {

    enum class DataModelTypes {
        kCameraDataModelType,
        kCameraIrDataModelType,
        kCameraCalibrationParamsDataModelType,
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
}