#include "algorithms/DepthMap.h"

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <visualizers/LidarVisualizer.h>
#include <data_models/local_map/LocalPosition.h>
#include "local_map/Frames.h"

namespace AutoDrive::Algorithms {


//    void DepthMap::onNewLidarData(std::shared_ptr<DataModels::LidarScanDataModel> data) {
//
//        auto lidarFrame = lidarIdentifierToFrame(data->getLidarIdentifier());
//        auto tf = context_.tfTree_.getTransformationForFrame(lidarFrame);
//        storeLidarDataInRootFrame(data,tf);
//    }


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

//        if(cameraId == DataLoader::CameraIndentifier::kCameraRightSide) {

        std::vector<cv::Point2f> valid2DPoints;
        std::vector<cv::Point3f> valid3DPoints{};
        getAllCurrentPointsProjectedToImage(cameraId, valid2DPoints, valid3DPoints, data->getImage().cols, data->getImage().rows, imuPose.toTf());

        auto img = data->getImage();
        if(!valid2DPoints.empty()) {
            for (auto &detection : data->getYoloDetections()) {

                auto detPointIndexes = getIndexesOfPointsInDetection(valid2DPoints, detection);

                for (auto &index : detPointIndexes) {
                    cv::circle(img, valid2DPoints.at(index), 5, {255, 0, 0});
                }

//                for (auto &p : valid2DPoints) {
//                    cv::circle(img, p, 5, {255, 0, 0});
//                }

                auto bb = detection->getBoundingBox();
                cv::rectangle(img, {static_cast<int>(bb.x1_), static_cast<int>(bb.y1_)},
                                   {static_cast<int>(bb.x2_), static_cast<int>(bb.y2_)}, {0, 0, 255}, 5);

                auto distance = getMedianDepthOfPointVector(valid3DPoints, detPointIndexes);

                DataModels::BoundingBox2D bbx {static_cast<float>(bb.x1_), static_cast<float>(bb.y1_), static_cast<float>(bb.x2_), static_cast<float>(bb.y2_)};
                DataModels::YoloDetection3D detection3D {bbx, distance, detection->getDetectionConfidence(), detection->getClassConfidence(), detection->getDetectionClass()};
                detection3D.addParent(data);
                //std::cout << "bb: " << detection3D.getBoundingBox().x1_ << " " << detection3D.getBoundingBox().y1_ << " " << detection3D.getBoundingBox().x2_ << " " << detection3D.getBoundingBox().y2_ << " " << detection3D.getDistance() << std::endl;
                output->push_back(detection3D);
            }
//            cv::imshow("bublebum", img);
//            cv::waitKey(100);
        //} else {
        //    context_.logger_.warning("Missing valid points for frustum distance estimation!");
        }

        return output;
    }


    void DepthMap::addProjector(std::shared_ptr<Projector> projector, DataLoader::CameraIndentifier id) {
        projectors_[id] = projector;
    }


    void DepthMap::storeLidarDataInRootFrame(std::shared_ptr<DataModels::LidarScanDataModel> data, rtl::Transformation3D<double>& tf) {

        auto scan = data->getScan();
        applyTransformOnPclData(*scan, lidarScans_[data->getLidarIdentifier()], tf);
    }


    void DepthMap::applyTransformOnPclData(pcl::PointCloud<pcl::PointXYZ>&input, pcl::PointCloud<pcl::PointXYZ>&output, rtl::Transformation3D<double>& tf) {
        auto rotMat = tf.rotQuaternion().rotMat();
        Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
        pcl_tf(0,0) = static_cast<float>(rotMat(0, 0)); pcl_tf(1,0) = static_cast<float>(rotMat(1, 0)); pcl_tf(2,0) = static_cast<float>(rotMat(2, 0));
        pcl_tf(0,1) = static_cast<float>(rotMat(0, 1)); pcl_tf(1,1) = static_cast<float>(rotMat(1, 1)); pcl_tf(2,1) = static_cast<float>(rotMat(2, 1));
        pcl_tf(0,2) = static_cast<float>(rotMat(0, 2)); pcl_tf(1,2) = static_cast<float>(rotMat(1, 2)); pcl_tf(2,2) = static_cast<float>(rotMat(2, 2));
        pcl_tf.translation() << tf.trX(), tf.trY(), tf.trZ();
        pcl::transformPointCloud (input, output, pcl_tf);
    }


    void DepthMap::getAllCurrentPointsProjectedToImage(
            DataLoader::CameraIndentifier id,
            std::vector<cv::Point2f>& validPoints2D,
            std::vector<cv::Point3f>& validPoints3D,
            size_t img_width,
            size_t img_height,
            rtl::Transformation3D<double> originToImu) {

        auto projector = projectors_[id];
        auto cameraFrame = cameraIdentifierToFrame(id);

        pcl::PointCloud<pcl::PointXYZ> destPCL;
        auto imuToCamera = context_.tfTree_.getTransformationForFrame(cameraFrame);
        rtl::Transformation3D<double> originToCameraTf = (imuToCamera.inverted()(originToImu.inverted()));//imuToCamera.inverted()(originToImu.inverted());

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

//        vis_.drawAggregatedPointcloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(test));
//        for(int j = 0 ; j < 360; j+=1) {
//            for (int i = 0; i < 360; i += 1) {
//                float angle = i * M_PI / 180;
//                points3D.emplace_back(cv::Point3f{sin(j*M_PI / 180), sin(angle), cos(angle)});
//            }
//        }

        projector->projectPoints(points3D, points2D);

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
        return 0;
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