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

#include <rtl/Transformation.h>

#include "Context.h"
#include "algorithms/Projector.h"

#include "data_models/local_map/FrustumDetection.h"
#include "data_models/camera/CameraIrFrameDataModel.h"
#include "data_models/yolo/YoloDetection.h"

namespace AtlasFusion::Algorithms {

    /**
     * Yolo Detection Reporojector is responsible for a reprojecting yolo (neural network generaly speaking] detections
     * on the RGB image into the plane of the given camera sensor. Usually IR camera.
     */
    class YoloDetectionReprojector {

    public:

        /**
         * Constructor
         * @param context global services provider (logging, calibrations, etc.)
         */
        YoloDetectionReprojector(Context& context)
        : context_{context} {

        }

        /**
         * Projects new NN's detections into the plane of a given camera
         * @param frustums 3D frustums detected by the neural network
         * @param currentCameraTf current camera position
         * @return 2D detection projected into camera's plain
         */
        std::shared_ptr<std::vector<DataModels::YoloDetection>> onNewDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> frustums, rtl::RigidTf3D<double> currentCameraTf);

        /**
         * Setter for an instance of the class that holds the internal camera calibration parameters
         * @param projector is able to projects 3D points into camera's plane
         */
        void setupCameraProjector(std::shared_ptr<Projector> projector) {cameraProjector_ = projector;}

    private:

        Context& context_;
        std::shared_ptr<Projector> cameraProjector_ = nullptr;

        long int framesCounter_ = -1;
        std::shared_ptr<DataModels::CameraIrFrameDataModel> lastFrame = nullptr;

    };
}

