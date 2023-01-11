#include "DataCache.h"

namespace AutoDrive{

    void DataCache::setNewIRFrame(const std::shared_ptr<DataModels::CameraIrFrameDataModel>& frame) {
        const auto id = frame->getCameraIdentifier();
        irFrames_.insert({id, frame});
    }

    std::shared_ptr<DataModels::CameraIrFrameDataModel> DataCache::getIRFrame(DataLoader::CameraIndentifier id) {
        if (irFrames_.find(id) != irFrames_.end()) {
            return irFrames_.at(id);
        }
        return nullptr;
    }

    void DataCache::setNewRGBFrame(const std::shared_ptr<DataModels::CameraFrameDataModel>& frame) {
        const auto id = frame->getCameraIdentifier();
        rgbFrames_.insert({id, frame});
    }

    std::shared_ptr<DataModels::CameraFrameDataModel> DataCache::getRGBFrame(DataLoader::CameraIndentifier id) {
        if (rgbFrames_.find(id) != rgbFrames_.end()) {
            return rgbFrames_.at(id);
        }
        return nullptr;
    }

    /* LiDARs */
    void DataCache::setNewLidarScan(const std::shared_ptr<DataModels::LidarScanDataModel>& frame) {
        lidarScans_[frame->getLidarIdentifier()] = frame;
    }

    std::shared_ptr<DataModels::LidarScanDataModel> DataCache::getLidarScan(DataLoader::LidarIdentifier id) {
        return lidarScans_[id];
    }

    /* Depth Map */
    void DataCache::setNewDepthMap(const std::shared_ptr<DataModels::CameraFrameDataModel>& map) {
        const auto id = map->getCameraIdentifier();
        depthMaps_.insert({id, map});
    }

    std::shared_ptr<DataModels::CameraFrameDataModel> DataCache::getDepthMap(DataLoader::CameraIndentifier id) {
        if (depthMaps_.find(id) != depthMaps_.end()) {
            return depthMaps_.at(id);
        }
        return nullptr;
    }

}