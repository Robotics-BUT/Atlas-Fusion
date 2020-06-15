#pragma once

#include <memory>
#include <rtl/Core.h>

#include "algorithms/Projector.h"
#include "data_models/local_map/YoloDetection3D.h"
#include "data_models/local_map/FrustumDetection.h"
#include "data_loader/DataLoaderIdentifiers.h"
#include "Context.h"

namespace AutoDrive::Algorithms {

    /**
     * Detections Processors hendles the data transformation from the raw 2D neural network detection with the estimated
     * depth into the unified 3D data model representation. Also transforms detections from the camera's local frame into
     * the central car frame
     */
    class DetectionsProcessor {

    public:

        /**
         * Constructor
         * @param context global services container (timestamps, logging, etc.)
         */
        explicit DetectionsProcessor(Context& context)
        : context_{context} {

        }

        /**
         * @param detections3D list of 2D NN detections with the estimated depth
         * @param frame camera identifier
         * @return list of 3D frustum data models
         */
        std::vector<std::shared_ptr<const DataModels::FrustumDetection>> onNew3DYoloDetections(std::shared_ptr<std::vector<DataModels::YoloDetection3D>> detections3D, std::string frame);

        /**
         * Inserter of a camera projector class instances
         * @param projector holds the camera calibration parameters for a given camera and is able to project 3D points
         * into 2D
         * @param id camera identifier
         */
        void addProjector(std::shared_ptr<Projector> projector, std::string id);

    private:

        Context& context_;
        std::map<std::string, std::shared_ptr<Projector>> projectors_{};

    };

}
