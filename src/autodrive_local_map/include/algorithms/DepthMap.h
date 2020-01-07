#pragma once

#include <rtl/Vector3D.h>

#include "Context.h"
#include "data_models/all.h"

#include "Projector.h"
#include "data_models/local_map/YoloDetection3D.h"

namespace AutoDrive::Algorithms {


    class DepthMap {

    public:

        explicit DepthMap(Context& context)
        : context_{context} {

        }

        void onNewLidarData(std::shared_ptr<DataModels::LidarScanDataModel>);
        std::shared_ptr<std::vector<DataModels::YoloDetection3D>> onNewCameraData(std::shared_ptr<DataModels::CameraFrameDataModel>);
        void addProjector(std::shared_ptr<Projector> projector, DataLoader::CameraIndentifier);

    private:

        Context& context_;
        std::map<DataLoader::CameraIndentifier, std::shared_ptr<Projector>> projectors_{};
        std::map<DataLoader::LidarIdentifier, pcl::PointCloud<pcl::PointXYZ>> lidarScans_{};

        void storeLidarDataInRootFrame(std::shared_ptr<DataModels::LidarScanDataModel> data, rtl::Transformation3D<double>& tf);
        void applyTransformOnPclData(pcl::PointCloud<pcl::PointXYZ>&input, pcl::PointCloud<pcl::PointXYZ>&output, rtl::Transformation3D<double>& tf);

        void getAllCurrentPointsProjectedToImage(DataLoader::CameraIndentifier id, std::vector<cv::Point2f>& validPoints2D, std::vector<cv::Point3f>& validPoints3D, size_t img_width, size_t img_height);

        std::vector<size_t> getIndexesOfPointsInDetection(std::vector<cv::Point2f>& validPoints2D, std::shared_ptr<DataModels::YoloDetection> detection);

        float getMedianDepthOfPointVector(std::vector<cv::Point3f>& points, std::vector<size_t>& indexes);
        std::string cameraIdentifierToFrame(DataLoader::CameraIndentifier id);
        std::string lidarIdentifierToFrame(DataLoader::LidarIdentifier id);

    };
}
