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

#include "data_models/local_map/FrustumDetection.h"
#include "data_models/local_map/LidarDetection.h"
#include "data_models/local_map/Object.h"

#include "Context.h"

namespace AtlasFusion::LocalMap {

    /**
     * Local Map is a container that aggregates information that defines created 3D map.
     */
    class LocalMap {

    public:

        /**
         * Constructor
         * @param context global services container (logger, etc.)
         */
        LocalMap(Context& context)
        : context_{context} {

        }

        /**
         * Setter for all the camera based detections
         * @param detections vector of camera based detections
         * @param sensorFrame frame that identifies camera frame that has been used for detection
         */
        void setFrustumDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> detections, std::string sensorFrame);

        /**
         * Setter for all point cloud based detections
         * @param detections lidar detections
         */
        void setLidarDetections(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections);

        /**
         * Setter for combinations of the point cloud based and camera based detections
         * @param objects vector of fused detections
         */
        void setObjects(std::vector<std::shared_ptr<DataModels::Object>> objects);

        /**
         * Getter for all camera based detections
         * @return camera based frustum detections
         */
        std::vector<std::shared_ptr<const DataModels::FrustumDetection>> getFrustumDetections();

        /**
         * Getter for all lidar based detections
         * @return point cloud based lidar detections
         */
        std::vector<std::shared_ptr<const DataModels::LidarDetection>> getLidarDetections();

        /**
         * Getter for all lidar-camera fused detections
         * @return lidar-camera fused detections
         */
        std::vector<std::shared_ptr<DataModels::Object>> getObjects();

        /**
         * Getter for all lidar-camera fused detections mapped into the lidar detection data format
         * @return mapped lidar-camera detections into the lidar based detection data model
         */
        std::vector<std::shared_ptr<const DataModels::LidarDetection>> getObjectsAsLidarDetections();

    private:
        Context& context_;

        std::map<std::string, std::vector<std::shared_ptr<const DataModels::FrustumDetection>>> frustumsDetections_{};
        std::vector<std::shared_ptr<const DataModels::LidarDetection>> lidarDetections_;
        std::vector<std::shared_ptr<DataModels::Object>> objects_;
    };
}

