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

#include "data_models/all.h"

#include "Projector.h"
#include "data_models/local_map/YoloDetection3D.h"
#include "data_models/local_map/PointCloudBatch.h"
#include "data_models/local_map/LocalPosition.h"
#include "algorithms/pointcloud/PointCloudProcessor.h"


namespace AutoDrive::Algorithms {

    /**
     *  Depth Map class covers the point cloud data projection into the given camera's sensor plane, and allows to
     *  estimate the distance in the object defined by the 2D bounding box. The distance estimation is done in the way,
     *  that Depth Map keeps set of 3D points, and when the 2D detection comes, it reprojects points into this
     *  2D bounding box and calculate the distance based on the statistics of the projected points.
     */
    class DepthMap {

    public:


        explicit DepthMap(Context &context, PointCloudProcessor &processor)
                : context_{context}, pointCloudProcessor_{processor} {}

        /**
         * Method receives the RGB camera frame that also contains neural network detections and projects point cloud
         * into these NN's detections. All the points that do not match any detections are removed, and based on the
         * matched points for each detection there is  the distance of the detection from the camera's frame estimated.
         * @param data The RGB image with all the parameters and the neural network detections
         * @param sensorCutoutPc aggregated point cloud cutout for the respective camera frame
         * @return The vector of detections wih estimated distances
         */
        std::vector<DataModels::YoloDetection3D>
        onNewCameraData(const std::shared_ptr<DataModels::CameraFrameDataModel> &data, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &sensorCutoutPc);

        /**
         * Adds a new instance of the Projector to the Depth Map's arsenal. These projectors are used to project point
         * cloud into the camera plain. The projector is bounded by the Camera Identifier to the specific camera.
         * @param projector Instance of the projector that Depth Map will remember
         * @param id unique identifier of the camera to which the projector is related
         */
        void addProjector(std::shared_ptr<Projector> projector, DataLoader::CameraIndentifier id);

        /**
         * Return all the points that fits into the given camera's FoV
         * @param id identifies which projector should be used to project point into the camera's frame
         * @param sensorCutoutPc aggregated point cloud cutout for the respective camera frame
         * @param imgWidth width of the frame in pixels
         * @param imgHeight height of the frame in pixels
         * @param useDistMat if should use the distortion matrix for the point cloud reprojection
         */
        std::shared_ptr<std::pair<std::vector<cv::Point2f>, std::vector<cv::Point3f>>> getPointsInCameraFoV(
                DataLoader::CameraIndentifier id,
                const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &sensorCutoutPc,
                size_t imgWidth,
                size_t imgHeight,
                bool useDistMat = true);

    private:

        Context &context_;
        PointCloudProcessor &pointCloudProcessor_;
        std::map<DataLoader::CameraIndentifier, std::shared_ptr<Projector>> projectors_{};

        void getAllCurrentPointsProjectedToImage(
                DataLoader::CameraIndentifier id,
                const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &sensorCutoutPc,
                std::vector<cv::Point2f> &validPoints2D,
                std::vector<cv::Point3f> &validPoints3D,
                size_t img_width,
                size_t img_height,
                bool useDistMat = true);

        std::vector<size_t> getIndexesOfPointsInDetection(const std::vector<cv::Point2f> &validPoints2D, const DataModels::YoloDetection &detection);

        float getMedianDepthOfPointVector(std::vector<cv::Point3f> &points, std::vector<size_t> &indexes);
    };
}
