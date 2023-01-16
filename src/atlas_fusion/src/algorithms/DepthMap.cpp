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

#include "algorithms/DepthMap.h"

#include <data_models/local_map/LocalPosition.h>

#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::Algorithms {

    std::vector<DataModels::YoloDetection3D>
    DepthMap::onNewCameraData(const std::shared_ptr<DataModels::CameraFrameDataModel> &data) {
        auto output = std::vector<DataModels::YoloDetection3D>();

        if (projectors_.count(data->getCameraIdentifier()) == 0) {
            context_.logger_.warning("Missing camera projector for DepthMap");
            return output;
        }

        auto cameraFrame = frameTypeFromIdentifier(data->getCameraIdentifier());
        auto cameraId = data->getCameraIdentifier();
        auto yoloDetections = data->getYoloDetections();

        std::vector<cv::Point2f> valid2DPoints{};
        std::vector<cv::Point3f> valid3DPoints{};

        getAllCurrentPointsProjectedToImage(cameraId, valid2DPoints, valid3DPoints, data->getImage().cols, data->getImage().rows);

        auto img = data->getImage();
        if (!valid2DPoints.empty()) {
            for (auto &detection: yoloDetections) {

                auto detPointIndexes = getIndexesOfPointsInDetection(valid2DPoints, detection);

                auto bb = detection.getBoundingBox();

                auto distance = getMedianDepthOfPointVector(valid3DPoints, detPointIndexes);
                if (distance > 0) {
                    DataModels::BoundingBox2D bbx{static_cast<float>(bb.x1_), static_cast<float>(bb.y1_),
                                                  static_cast<float>(bb.x2_), static_cast<float>(bb.y2_)};
                    DataModels::YoloDetection3D detection3D{bbx, distance, detection.getDetectionConfidence(),
                                                            detection.getDetectionClass()};
                    detection3D.addParent(data);
                    output.push_back(detection3D);
                }
            }
        } else {
            context_.logger_.warning("No valid points for estimating YOLO detection depth.");
        }

        return output;
    }


    void DepthMap::addProjector(std::shared_ptr<Projector> projector, DataLoader::CameraIndentifier id) {
        projectors_[id] = std::move(projector);
    }


    std::shared_ptr<std::pair<std::vector<cv::Point2f>, std::vector<cv::Point3f>>>
    DepthMap::getPointsInCameraFoV(DataLoader::CameraIndentifier id, size_t imgWidth, size_t imgHeight, bool useDistMat) {

        auto output = std::make_shared<std::pair<std::vector<cv::Point2f>, std::vector<cv::Point3f>>>();
        getAllCurrentPointsProjectedToImage(id, output->first, output->second, imgWidth, imgHeight, useDistMat);
        return output;
    }

    void DepthMap::getAllCurrentPointsProjectedToImage(
            DataLoader::CameraIndentifier id,
            std::vector<cv::Point2f> &validPoints2D,
            std::vector<cv::Point3f> &validPoints3D,
            size_t img_width,
            size_t img_height,
            bool useDistMat) {

        auto projector = projectors_[id];
        auto cameraFrame = frameTypeFromIdentifier(id);

        pcl::PointCloud<pcl::PointXYZ>::Ptr destPCL;
        auto imuToCamera = context_.tfTree_.getTransformationForFrame(cameraFrame);
        rtl::RigidTf3D<double> originToCameraTf = imuToCamera.inverted();

        //TODO: Takes around 10 ms because of lots of points being transformed at once
        // transformation of all aggregated points takes place for every camera every frame -> is there some possible room for improvement?
        destPCL = pointCloudProcessor_.transformPointCloud(aggregatedPointCloud_, originToCameraTf);

        std::vector<cv::Point3f> points3D;
        std::vector<cv::Point2f> points2D;
        points3D.reserve(destPCL->size());

        for (const auto &pnt: destPCL->points) {
            if (pnt.z > 0) {
                points3D.emplace_back(pnt.x, pnt.y, pnt.z);
            }
        }

        projector->projectPoints(points3D, points2D, useDistMat);

        if (points3D.size() != points3D.size()) {
            context_.logger_.error("Number of projected points does not correspond with number of input points!");
        }

        validPoints2D.reserve(points2D.size());
        validPoints3D.reserve(points3D.size());

        for (size_t i = 0; i < points3D.size(); i++) {
            if (points2D.at(i).y >= 0 && points2D.at(i).y < float(img_height) && points2D.at(i).x >= 0 && points2D.at(i).x < float(img_width)) {
                validPoints2D.push_back(points2D.at(i));
                validPoints3D.push_back(points3D.at(i));
            }
        }
    }


    std::vector<size_t> DepthMap::getIndexesOfPointsInDetection(const std::vector<cv::Point2f> &validPoints2D, const DataModels::YoloDetection &detection) {
        std::vector<size_t> output;
        for (size_t i = 0; i < validPoints2D.size(); i++) {
            const auto &point = validPoints2D.at(i);
            const auto &bb = detection.getBoundingBox();
            if (point.x > bb.x1_ && point.x < bb.x2_ && point.y > bb.y1_ && point.y < bb.y2_) {
                output.push_back(i);
            }
        }
        return output;
    }


    float DepthMap::getMedianDepthOfPointVector(std::vector<cv::Point3f> &points, std::vector<size_t> &indexes) {
        std::vector<float> distances;
        distances.reserve(points.size());

        for (const auto &index: indexes) {
            distances.push_back(points.at(index).z);
        }

        if (distances.size() > 0) {
            std::sort(distances.begin(), distances.end());
            auto midIndex = static_cast<size_t>(distances.size() / 2);
            return distances.at(midIndex);
        }
        return -1;
    }
}