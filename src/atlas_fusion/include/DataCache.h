#pragma once

#include "data_models/all.h"
#include "data_loader/DataLoaderIdentifiers.h"

namespace AutoDrive {

    class DataCache {

    public:

        DataCache() = default;

        /* IR Cameras */
        void setNewIRFrame(std::shared_ptr<DataModels::CameraIrFrameDataModel> frame);
        std::shared_ptr<DataModels::CameraIrFrameDataModel> getIRFrame(DataLoader::CameraIndentifier id);
        size_t getIRFrameNo(DataLoader::CameraIndentifier id);

        /* RGB Cameras */
        void setNewRGBFrame(std::shared_ptr<DataModels::CameraFrameDataModel> frame);
        std::shared_ptr<DataModels::CameraFrameDataModel> getRGBFrame(DataLoader::CameraIndentifier id);
        size_t getRGBFrameNo(DataLoader::CameraIndentifier id);

        /* LiDARs */
        void setNewLidarScan(std::shared_ptr<DataModels::LidarScanDataModel> frame);
        std::shared_ptr<DataModels::LidarScanDataModel> getLidarScan(DataLoader::LidarIdentifier id);
        size_t getLidarScanNo(DataLoader::LidarIdentifier id);

        /* Depth Map */
        void setNewDepthMap(std::shared_ptr<DataModels::CameraFrameDataModel> map);
        std::shared_ptr<DataModels::CameraFrameDataModel> getDepthMap(DataLoader::CameraIndentifier id);
        size_t getDepthMapScanNo(DataLoader::CameraIndentifier id);

    private:

        std::map<DataLoader::CameraIndentifier, const std::shared_ptr<DataModels::CameraIrFrameDataModel>> irFrames_;
        std::map<DataLoader::CameraIndentifier, int> irFramesCounter_;

        std::map<DataLoader::CameraIndentifier, const std::shared_ptr<DataModels::CameraFrameDataModel>> rgbFrames_;
        std::map<DataLoader::CameraIndentifier, int> rgbFramesCounter_;

        std::map<DataLoader::LidarIdentifier, const std::shared_ptr<DataModels::LidarScanDataModel>> lidarScans_;
        std::map<DataLoader::LidarIdentifier, int> lidarScansCounter_;

        std::map<DataLoader::CameraIndentifier, const std::shared_ptr<DataModels::CameraFrameDataModel>> depthMaps_;
        std::map<DataLoader::CameraIndentifier, int> depthMapsCounter_;
    };

}