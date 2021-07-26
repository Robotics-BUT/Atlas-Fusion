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

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <visualizers/LidarVisualizer.h>
#include <data_models/local_map/LocalPosition.h>
#include "local_map/Frames.h"

namespace AtlasFusion::Algorithms {


    void DepthMap::updatePointcloudData(std::vector<std::shared_ptr<DataModels::PointCloudBatch>> batches) {
        batches_ = batches;
    }

    std::shared_ptr<std::vector<DataModels::YoloDetection3D>> DepthMap::onNewCameraData(std::shared_ptr<DataModels::CameraFrameDataModel> data, DataModels::LocalPosition imuPose) {

        auto output = std::make_shared<std::vector<DataModels::YoloDetection3D>>();

        // project all point into specific camera
        if(projectors_.count(data->getCameraIdentifier()) == 0){
            context_.logger_.warning("Missing camera projector for DepthMap");
            return output;
        }

        auto cameraFrame = cameraIdentifierToFrame(data->getCameraIdentifier());
        auto cameraId = data->getCameraIdentifier();
        auto yoloDetections = data->getYoloDetections();


        std::vector<cv::Point2f> valid2DPoints;
        std::vector<cv::Point3f> valid3DPoints{};
        getAllCurrentPointsProjectedToImage(cameraId, valid2DPoints, valid3DPoints, data->getImage().cols, data->getImage().rows, imuPose.toTf());

        auto img = data->getImage();
        if(!valid2DPoints.empty()) {
            for (auto &detection : data->getYoloDetections()) {

                auto detPointIndexes = getIndexesOfPointsInDetection(valid2DPoints, detection);

                auto bb = detection->getBoundingBox();

                auto distance = getMedianDepthOfPointVector(valid3DPoints, detPointIndexes);
                if(distance > 0) {
                    DataModels::BoundingBox2D bbx{static_cast<float>(bb.x1_), static_cast<float>(bb.y1_),
                                                  static_cast<float>(bb.x2_), static_cast<float>(bb.y2_)};
                    DataModels::YoloDetection3D detection3D{bbx, distance, detection->getDetectionConfidence(),
                                                            detection->getDetectionClass()};
                    detection3D.addParent(data);
                    output->push_back(detection3D);
                }
            }
        }else {
            context_.logger_.warning("No valid points for estimating YOLO detection depth.");
        }

        return output;
    }


    void DepthMap::addProjector(std::shared_ptr<Projector> projector, DataLoader::CameraIndentifier id) {
        projectors_[id] = projector;
    }


    std::shared_ptr<std::pair<std::vector<cv::Point2f>, std::vector<cv::Point3f>>> DepthMap::getPointsInCameraFoV(
            DataLoader::CameraIndentifier id,
            size_t imgWidth,
            size_t imgHeight,
            rtl::RigidTf3D<double> currentFrameTf,
            bool useDistMat) {

        auto output = std::make_shared<std::pair<std::vector<cv::Point2f>, std::vector<cv::Point3f>>>();
        getAllCurrentPointsProjectedToImage(id, output->first, output->second, imgWidth, imgHeight, currentFrameTf, useDistMat);
        return output;
    }


    void DepthMap::storeLidarDataInRootFrame(std::shared_ptr<DataModels::LidarScanDataModel> data, rtl::RigidTf3D<double>& tf) {

        auto scan = data->getScan();
        applyTransformOnPclData(*scan, lidarScans_[data->getLidarIdentifier()], tf);
    }


    void DepthMap::applyTransformOnPclData(pcl::PointCloud<pcl::PointXYZ>&input, pcl::PointCloud<pcl::PointXYZ>&output, rtl::RigidTf3D<double>& tf) {
        auto rotMat = tf.rotQuaternion().rotMat();
        Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
        pcl_tf(0,0) = static_cast<float>(rotMat(0, 0)); pcl_tf(1,0) = static_cast<float>(rotMat(1, 0)); pcl_tf(2,0) = static_cast<float>(rotMat(2, 0));
        pcl_tf(0,1) = static_cast<float>(rotMat(0, 1)); pcl_tf(1,1) = static_cast<float>(rotMat(1, 1)); pcl_tf(2,1) = static_cast<float>(rotMat(2, 1));
        pcl_tf(0,2) = static_cast<float>(rotMat(0, 2)); pcl_tf(1,2) = static_cast<float>(rotMat(1, 2)); pcl_tf(2,2) = static_cast<float>(rotMat(2, 2));
        pcl_tf.translation() << tf.trVecX(), tf.trVecY(), tf.trVecZ();
        pcl::transformPointCloud (input, output, pcl_tf);
    }


    void DepthMap::getAllCurrentPointsProjectedToImage(
            DataLoader::CameraIndentifier id,
            std::vector<cv::Point2f>& validPoints2D,
            std::vector<cv::Point3f>& validPoints3D,
            size_t img_width,
            size_t img_height,
            rtl::RigidTf3D<double> originToImu,
            bool useDistMat) {

        auto projector = projectors_[id];
        auto cameraFrame = cameraIdentifierToFrame(id);

        pcl::PointCloud<pcl::PointXYZ> destPCL;
        auto imuToCamera = context_.tfTree_.getTransformationForFrame(cameraFrame);
        rtl::RigidTf3D<double> originToCameraTf = (imuToCamera.inverted()(originToImu.inverted()));//imuToCamera.inverted()(originToImu.inverted());

        for(const auto& batch : batches_) {
            destPCL += *(batch->getTransformedPointsWithAnotherTF(originToCameraTf));
        }

        std::vector<cv::Point3f> points3D;
        std::vector<cv::Point2f> points2D;
        points3D.reserve(destPCL.width * destPCL.height);

        pcl::PointCloud<pcl::PointXYZ> test;
        for(const auto& pnt : destPCL ){
            if(pnt.z > 0 ) {
                points3D.push_back({pnt.x, pnt.y, pnt.z});
                test.push_back({pnt.x, pnt.y, pnt.z});
            }
        }

        projector->projectPoints(points3D, points2D, useDistMat);

        if(points2D.size() != points3D.size()) {
            context_.logger_.error("Number of projected points does not corresponds with number of input points!");
        }

        validPoints2D.reserve(points2D.size());
        validPoints3D.reserve(points3D.size());

        for(size_t i = 0 ; i < points3D.size() ; i++) {
            if(points2D.at(i).y >= 0 && points2D.at(i).y < img_height && points2D.at(i).x >= 0 && points2D.at(i).x < img_width) {
                validPoints2D.push_back(points2D.at(i));
                validPoints3D.push_back(points3D.at(i));
            }
        }
    }


    std::vector<size_t> DepthMap::getIndexesOfPointsInDetection(std::vector<cv::Point2f>& validPoints2D, std::shared_ptr<DataModels::YoloDetection> detection) {
        std::vector<size_t> output;
        for(size_t i = 0 ; i < validPoints2D.size() ; i++) {
            const auto& point = validPoints2D.at(i);
            const auto& bb = detection->getBoundingBox();
            if(point.x > bb.x1_ && point.x < bb.x2_ && point.y > bb.y1_ && point.y < bb.y2_){
                output.push_back(i);
            }
        }
        return output;
    }


    float DepthMap::getMedianDepthOfPointVector(std::vector<cv::Point3f>& points, std::vector<size_t>& indexes) {
        std::vector<float> distances;
        distances.reserve(points.size());

        for(const auto& index : indexes) {
            distances.push_back(points.at(index).z);
        }

        if(distances.size() > 0) {
            std::sort(distances.begin(), distances.end());
            auto midIndex = static_cast<size_t>(distances.size() / 2);
            return distances.at(midIndex);
        }
        return -1;
    }


    std::string DepthMap::cameraIdentifierToFrame(DataLoader::CameraIndentifier id) {
        switch(id){
            case DataLoader::CameraIndentifier::kCameraLeftFront:
                return LocalMap::Frames::kCameraLeftFront;
            case DataLoader::CameraIndentifier::kCameraLeftSide:
                return LocalMap::Frames::kCameraLeftSide;
            case DataLoader::CameraIndentifier::kCameraRightFront:
                return LocalMap::Frames::kCameraRightFront;
            case DataLoader::CameraIndentifier::kCameraRightSide:
                return LocalMap::Frames::kCameraRightSide;
            case DataLoader::CameraIndentifier::kCameraIr:
                return LocalMap::Frames::kCameraIr;
            case DataLoader::CameraIndentifier::kCameraVirtual:
                return LocalMap::Frames::kCameraVirtual;
            default:
                context_.logger_.warning("Unexpected camera ID type in depth map");
                return LocalMap::Frames::kOrigin;
        }
    }


    std::string DepthMap::lidarIdentifierToFrame(DataLoader::LidarIdentifier id) {
        switch (id){
            case DataLoader::LidarIdentifier::kRightLidar:
                return LocalMap::Frames::kLidarRight;
            case DataLoader::LidarIdentifier::kLeftLidar:
                return LocalMap::Frames::kLidarLeft;
            default:
                context_.logger_.warning("Unexpected lidar ID type in depth map");
                return LocalMap::Frames::kOrigin;
        }
    }
}