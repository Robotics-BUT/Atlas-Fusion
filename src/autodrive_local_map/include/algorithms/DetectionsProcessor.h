#pragma once

#include <memory>
#include <rtl/Frustum3D.h>

#include "algorithms/Projector.h"
#include "data_models/local_map/YoloDetection3D.h"
#include "data_models/local_map/FrustumDetection.h"
#include "data_loader/DataLoaderIdentifiers.h"
#include "Context.h"

namespace AutoDrive::Algorithms {

    class DetectionsProcessor {

    public:

        explicit DetectionsProcessor(Context& context)
        : context_{context} {

        }

        std::vector<std::shared_ptr<DataModels::FrustumDetection>> onNew3DYoloDetections(std::shared_ptr<std::vector<DataModels::YoloDetection3D>>, std::string);
        void addProjector(std::shared_ptr<Projector> projector, std::string);

    private:

        Context& context_;
        std::map<std::string, std::shared_ptr<Projector>> projectors_{};

    };

}
