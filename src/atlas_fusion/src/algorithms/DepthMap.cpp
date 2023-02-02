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

#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::Algorithms {

    std::vector<DataModels::YoloDetection3D>
    DepthMap::onNewCameraData(const std::shared_ptr<DataModels::CameraFrameDataModel> &data, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &sensorCutoutPc) {
        auto output = std::vector<DataModels::YoloDetection3D>();

        if (projectors_.count(data->getCameraIdentifier()) == 0) {
            context_.logger_.warning("Missing camera projector for DepthMap");
            return output;
        }

        auto cameraId = data->getCameraIdentifier();
        auto yoloDetections = data->getYoloDetections();

        std::vector<cv::Point2f> valid2DPoints{};
        std::vector<cv::Point3f> valid3DPoints{};

        getAllCurrentPointsProjectedToImage(cameraId, sensorCutoutPc, valid2DPoints, valid3DPoints, data->getImage().cols, data->getImage().rows);

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
    DepthMap::getPointsInCameraFoV(DataLoader::CameraIndentifier id, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &sensorCutoutPc, size_t imgWidth,
                                   size_t imgHeight, bool useDistMat) {
        auto output = std::make_shared<std::pair<std::vector<cv::Point2f>, std::vector<cv::Point3f>>>();
        getAllCurrentPointsProjectedToImage(id, sensorCutoutPc, output->first, output->second, imgWidth, imgHeight, useDistMat);
        return output;
    }

    void DepthMap::getAllCurrentPointsProjectedToImage(
            DataLoader::CameraIndentifier id,
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &sensorCutoutPc,
            std::vector<cv::Point2f> &validPoints2D,
            std::vector<cv::Point3f> &validPoints3D,
            size_t img_width,
            size_t img_height,
            bool useDistMat) {
        Timer t("Get all current points projected to image: " + frameTypeName(frameTypeFromIdentifier(id)), 0);

        auto projector = projectors_[id];
        auto cameraFrame = frameTypeFromIdentifier(id);

        pcl::PointCloud<pcl::PointXYZ>::Ptr destPc;
        auto imuToCamera = context_.tfTree_.getTransformationForFrame(cameraFrame);
        rtl::RigidTf3D<double> originToCameraTf = imuToCamera.inverted();

        destPc = pointCloudProcessor_.transformPointCloud(sensorCutoutPc, originToCameraTf);

        std::vector<cv::Point3f> points3D;
        std::vector<cv::Point2f> points2D;
        points3D.reserve(destPc->size());

        for (const auto &pnt: destPc->points) {
            if (pnt.z > -.5f) {
                points3D.emplace_back(pnt.x, pnt.y, pnt.z);
            }
        }

        projector->projectPoints(points3D, points2D, useDistMat);

        validPoints2D.reserve(points2D.size());
        validPoints3D.reserve(points3D.size());

        for (size_t i = 0; i < points3D.size(); i++) {
            const auto& p = points2D.at(i);
            if (p.y >= 0 && p.y < float(img_height) && p.x >= 0 && p.x < float(img_width)) {
                validPoints2D.push_back(points2D.at(i));
                validPoints3D.push_back(points3D.at(i));
            }
        }
    }


    std::vector<size_t> DepthMap::getIndexesOfPointsInDetection(const std::vector<cv::Point2f> &validPoints2D, const DataModels::YoloDetection &detection) {
        std::vector<size_t> output;

        const auto &bb = detection.getBoundingBox();
        for (size_t i = 0; i < validPoints2D.size(); i++) {
            const auto &p = validPoints2D.at(i);
            if (p.x > bb.x1_ && p.x < bb.x2_ && p.y > bb.y1_ && p.y < bb.y2_) {
                output.push_back(i);
            }
        }
        return output;
    }


    float DepthMap::getMedianDepthOfPointVector(const std::vector<cv::Point3f> &points, std::vector<size_t> &indexes) {
        std::vector<float> distances;
        distances.reserve(points.size());

        for (const auto &index: indexes) {
            const auto& p= points.at(index);
            distances.push_back(sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2)));
        }

        if (distances.size() > 0) {
            std::sort(distances.begin(), distances.end());
            auto midIndex = static_cast<size_t>(distances.size() / 2);
            return distances.at(midIndex);
        }
        return -1;
    }
}