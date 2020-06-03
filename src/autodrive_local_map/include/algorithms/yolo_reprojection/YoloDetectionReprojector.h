#pragma once

#include <rtl/Transformation3D.h>

#include "Context.h"
#include "algorithms/Projector.h"

#include "data_models/local_map/FrustumDetection.h"
#include "data_models/camera/CameraIrFrameDataModel.h"
#include "data_models/yolo/YoloDetection.h"

namespace AutoDrive::Algorithms {

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
         * @return 2D detection projected into cmaera's plain
         */
        std::shared_ptr<std::vector<DataModels::YoloDetection>> onNewDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> frustums, rtl::Transformation3D<double> currentCameraTf);

        /**
         * Setter for an instance of the class that holds the internal camera calibration parameters
         * @param projector is able to projects 3D points into camera's plane
         */
        void setupCameraProjector(std::shared_ptr<Projector> projector) {cameraProjector_ = projector;}

        /**
         * Method counts the number of IR frames that have passed during the mapping session and hold the frame in the
         * memory
         * @param frame IR camera frame
         */
        void onNewIRFrame(std::shared_ptr<DataModels::CameraIrFrameDataModel> frame);

        /**
         * Getter for number of IR frames passed during the mapping session
         * @return IR frame number
         */
        size_t getCurrentIrFrameNo() const;

        /**
         * Getter for last IR frame holded by the class
         * @return IR frame
         */
        std::shared_ptr<DataModels::CameraIrFrameDataModel> getLastIrFrame() const;

        /**
         * Getter for dimensions of the IR frame
         * @return width and height of the IR frame
         */
        std::pair<int, int> getLastIrFrameWidthHeight() const;

    private:

        Context& context_;
        std::shared_ptr<Projector> cameraProjector_ = nullptr;

        size_t framesCounter_ = -1;
        std::shared_ptr<DataModels::CameraIrFrameDataModel> lastFrame = nullptr;

    };
}

