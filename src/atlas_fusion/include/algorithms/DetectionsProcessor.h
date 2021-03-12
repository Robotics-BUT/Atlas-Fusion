/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <memory>
#include <rtl/Core.h>

#include "algorithms/Projector.h"
#include "data_models/local_map/YoloDetection3D.h"
#include "data_models/local_map/FrustumDetection.h"
#include "data_loader/DataLoaderIdentifiers.h"
#include "Context.h"

namespace AtlasFusion::Algorithms {

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
