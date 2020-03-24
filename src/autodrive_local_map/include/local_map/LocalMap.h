#pragma once

#include "data_models/local_map/FrustumDetection.h"
#include "data_models/local_map/LidarDetection.h"
#include "data_models/local_map/Object.h"

#include "Context.h"

namespace AutoDrive::LocalMap {

    class LocalMap {

    public:

        LocalMap(Context& context)
        : context_{context} {

        }

        void setFrustumDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>>, std::string sensorFrame);
        void setLidarDetections(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections);
        void setObjects(std::vector<std::shared_ptr<DataModels::Object>> objects);

        std::vector<std::shared_ptr<const DataModels::FrustumDetection>> getFrustumDetections();
        std::vector<std::shared_ptr<const DataModels::LidarDetection>> getLidarDetections();
        std::vector<std::shared_ptr<DataModels::Object>> getObjects();

        std::vector<std::shared_ptr<const DataModels::LidarDetection>> getObjectsAsLidarDetections();

    private:
        Context& context_;

        std::map<std::string, std::vector<std::shared_ptr<const DataModels::FrustumDetection>>> frustumsDetections_{};
        std::vector<std::shared_ptr<const DataModels::LidarDetection>> lidarDetections_;
        std::vector<std::shared_ptr<DataModels::Object>> objects_;
    };
}

