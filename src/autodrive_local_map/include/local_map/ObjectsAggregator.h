#pragma once

#include "Context.h"
#include "data_models/local_map/LidarDetection.h"

namespace AutoDrive::LocalMap {

    class ObjectsAggregator {

    public:

        ObjectsAggregator() = delete;

        ObjectsAggregator(Context& context)
        : context_{context} {

        }

        std::vector<std::shared_ptr<const DataModels::LidarDetection>> aggregateLidarDetections(
                std::vector<std::shared_ptr<const DataModels::LidarDetection>> previousDetections,
                std::vector<std::shared_ptr<const DataModels::LidarDetection>> newDetections) const;

    private:

        std::vector<std::pair<unsigned, unsigned>> matchDetections(
                std::vector<std::shared_ptr<const DataModels::LidarDetection>> a,
                std::vector<std::shared_ptr<const DataModels::LidarDetection>> b) const;

        std::vector<std::shared_ptr<const DataModels::LidarDetection>> mergeDetections(
                std::vector<std::shared_ptr<const DataModels::LidarDetection>> a,
                std::vector<std::shared_ptr<const DataModels::LidarDetection>> b,
                std::vector<std::pair<unsigned, unsigned>> matches) const;

        Context& context_;

    };
}


