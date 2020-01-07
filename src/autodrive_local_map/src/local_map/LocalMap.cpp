#include "local_map/LocalMap.h"

namespace AutoDrive::LocalMap {


    void LocalMap::onNewFrustumDetections(std::vector<std::shared_ptr<DataModels::FrustumDetection>> detections, std::string sensorFrame) {
        frustumsDetections_[sensorFrame] = detections;
    }


    std::vector<std::shared_ptr<DataModels::FrustumDetection>> LocalMap::getAllFrustumDetections() {

        std::vector<std::shared_ptr<DataModels::FrustumDetection>> output;
        for(auto it = frustumsDetections_.begin(); it != frustumsDetections_.end(); it++){
            for(auto& frustum : it->second) {
                output.push_back(frustum);
            }
        }
        return output;
    }

}