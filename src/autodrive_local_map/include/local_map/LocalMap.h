#pragma once

#include "data_models/local_map/FrustumDetection.h"

#include "Context.h"

namespace AutoDrive::LocalMap {

    class LocalMap {

    public:

        LocalMap(Context& context)
        : context_{context} {

        }

        void onNewFrustumDetections(std::vector<std::shared_ptr<DataModels::FrustumDetection>>, std::string sensorFrame);
        std::vector<std::shared_ptr<DataModels::FrustumDetection>> getAllFrustumDetections();

    private:
        Context& context_;

        std::map<std::string, std::vector<std::shared_ptr<DataModels::FrustumDetection>>> frustumsDetections_{};
    };
}

