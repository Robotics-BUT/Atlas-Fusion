#include "local_map/LocalMap.h"

namespace AutoDrive::LocalMap {


    void LocalMap::setFrustumDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> detections, std::string sensorFrame) {
        frustumsDetections_[sensorFrame] = detections;
    }



    void LocalMap::setLidarDetections(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections) {
        lidarDetections_ = detections;
    }


    void LocalMap::setObjects(std::vector<std::shared_ptr<DataModels::Object>> objects) {
        objects_ = objects;
    }


    std::vector<std::shared_ptr<const DataModels::LidarDetection>> LocalMap::getObjectsAsLidarDetections() {

        std::vector<std::shared_ptr<const DataModels::LidarDetection>> output;

        return output;
    }


    std::vector<std::shared_ptr<const DataModels::FrustumDetection>> LocalMap::getFrustumDetections() {

        std::vector<std::shared_ptr<const DataModels::FrustumDetection>> output;
        for(auto it = frustumsDetections_.begin(); it != frustumsDetections_.end(); it++){
            for(auto& frustum : it->second) {
                output.push_back(frustum);
            }
        }
        return output;
    }



    std::vector<std::shared_ptr<const DataModels::LidarDetection>> LocalMap::getLidarDetections() {
        return lidarDetections_;
    }


    std::vector<std::shared_ptr<DataModels::Object>> LocalMap::getObjects() {
        return objects_;
    }

}