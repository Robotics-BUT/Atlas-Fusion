#include "DataCache.h"
#include "RustStyle.h"

namespace AtlasFusion{

    void DataCache::setNewIRFrame(const std::shared_ptr<DataModels::CameraIrFrameDataModel> frame) {
        let id = frame->getCameraIdentifier();

        if (irFrames_.count(id) == 0) {
            irFrames_.insert({id, frame});
        }
        irFrames_.at(id) = frame;

        if (irFramesCounter_.count(id) == 0) {
            irFramesCounter_[id] = 0;
        } else {
            irFramesCounter_.at(id) += 1;
        }
    }

    std::shared_ptr<DataModels::CameraIrFrameDataModel> DataCache::getIRFrame(DataLoader::CameraIndentifier id) {
        if (irFrames_.find(id) != irFrames_.end()) {
            return irFrames_.at(id);
        }
        return nullptr;
    }

    int DataCache::getIRFrameNo(DataLoader::CameraIndentifier id) {
        if (irFramesCounter_.count(id) == 0) {
            return -1;
        }
        return irFramesCounter_.at(id);
    }


    void DataCache::setNewRGBFrame(std::shared_ptr<DataModels::CameraFrameDataModel> frame) {
        let id = frame->getCameraIdentifier();

        if (rgbFrames_.count(id) == 0) {
            rgbFrames_.insert({id, frame});
        }
        rgbFrames_.at(id) = frame;


        if (rgbFramesCounter_.count(id) == 0) {
            rgbFramesCounter_[id] = 0;
        } else {
            rgbFramesCounter_.at(id) += 1;
        }
    }

    std::shared_ptr<DataModels::CameraFrameDataModel> DataCache::getRGBFrame(DataLoader::CameraIndentifier id) {
        if (rgbFrames_.find(id) != rgbFrames_.end()) {
            return rgbFrames_.at(id);
        }
        return nullptr;
    }

    int DataCache::getRGBFrameNo(DataLoader::CameraIndentifier id) {
        if (rgbFramesCounter_.count(id) == 0) {
            return -1;
        }
        return rgbFramesCounter_.at(id);
    }

    /* LiDARs */
    void DataCache::setNewLidarScan(std::shared_ptr<DataModels::LidarScanDataModel> scan) {
        let id = scan->getLidarIdentifier();

        if (lidarScans_.count(id) == 0) {
            lidarScans_.insert({id, scan});
        }
        lidarScans_.at(id) = scan;

        if (lidarScansCounter_.count(id) == 0) {
            lidarScansCounter_[id] = 0;
        } else {
            lidarScansCounter_.at(id) += 1;
        }
    }

    std::shared_ptr<DataModels::LidarScanDataModel> DataCache::getLidarScan(DataLoader::LidarIdentifier id) {
        if (lidarScans_.find(id) != lidarScans_.end()) {
            return lidarScans_.at(id);
        }
        return nullptr;
    }

    int DataCache::getLidarScanNo(DataLoader::LidarIdentifier id) {
        if (lidarScansCounter_.count(id) == 0) {
            return -1;
        }
        return lidarScansCounter_.at(id);
    }

    /* Depth Map */
    void DataCache::setNewDepthMap(std::shared_ptr<DataModels::DepthMapDataModel> map) {
        let id = map->getCameraIdentifier();

        if (depthMaps_.count(id) == 0) {
            depthMaps_.insert({id, map});
        }
        depthMaps_.at(id) = map;

        if (depthMapsCounter_.count(id) == 0) {
            depthMapsCounter_[id] = 0;
        } else {
            depthMapsCounter_.at(id) += 1;
        }
    }

    std::shared_ptr<DataModels::DepthMapDataModel> DataCache::getDepthMap(DataLoader::CameraIndentifier id) {
        if (depthMaps_.find(id) != depthMaps_.end()) {
            return depthMaps_.at(id);
        }
        return nullptr;
    }

    int DataCache::getDepthMapNo(DataLoader::CameraIndentifier id) {
        if (depthMapsCounter_.count(id) == 0) {
            return -1;
        }
        return depthMapsCounter_.at(id);
    }
}