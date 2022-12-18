#pragma once

#include "data_models/all.h"
#include "data_loader/DataLoaderIdentifiers.h"

namespace AutoDrive {

    class DataCache {

    public:

        DataCache() = default;

        /* IR Cameras */
        void setNewIRFrame(const std::shared_ptr<DataModels::CameraIrFrameDataModel>& frame);
        std::shared_ptr<DataModels::CameraIrFrameDataModel> getIRFrame(DataLoader::CameraIndentifier id);

        /* RGB Cameras */
        void setNewRGBFrame(const std::shared_ptr<DataModels::CameraFrameDataModel>& frame);
        std::shared_ptr<DataModels::CameraFrameDataModel> getRGBFrame(DataLoader::CameraIndentifier id);

        /* LiDARs */
        void setNewLidarScan(const std::shared_ptr<DataModels::LidarScanDataModel>& frame);
        std::shared_ptr<DataModels::LidarScanDataModel> getLidarScan(DataLoader::LidarIdentifier id);

        /* Depth Map */
        void setNewDepthMap(const std::shared_ptr<DataModels::CameraFrameDataModel>& map);
        std::shared_ptr<DataModels::CameraFrameDataModel> getDepthMap(DataLoader::CameraIndentifier id);

    private:

        std::map<DataLoader::CameraIndentifier, std::shared_ptr<DataModels::CameraIrFrameDataModel>> irFrames_;
        std::map<DataLoader::CameraIndentifier, std::shared_ptr<DataModels::CameraFrameDataModel>> rgbFrames_;
        std::map<DataLoader::LidarIdentifier, std::shared_ptr<DataModels::LidarScanDataModel>> lidarScans_;
        std::map<DataLoader::CameraIndentifier, std::shared_ptr<DataModels::CameraFrameDataModel>> depthMaps_;
    };

}