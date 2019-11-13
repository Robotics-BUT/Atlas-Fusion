#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "data_loader/data_models/imu/ImuImuDataModel.h"
#include "data_loader/data_models/camera/CameraFrameDataModel.h"
#include "data_loader/data_models/camera/CameraIrFrameDataModel.h"
#include "data_loader/data_models/DataModelTypes.h"

#include "Topics.h"
#include "LogService.h"
#include "TFTree.h"

#include "LidarVisualizer.h"
#include "ImuVisualizer.h"
#include "CameraVisualizer.h"
#include "GnssVisualizer.h"
#include "TFVisualizer.h"

namespace AutoDrive::Visualizers {

    class VisualizationHandler {

    public:

        explicit VisualizationHandler(ros::NodeHandle& n, TFTree& tfTree, LogService& logger)
        : node_(n)
        , logger_(logger)
        , lidarVisualizer_(n, logger)
        , imuVisualizer_(n, logger)
        , cameraVisualizer_(n, logger)
        , gnssVisualizer_(n, logger)
        , tfTreeVisualizer_{tfTree, logger}{
            testCubePublisher_ = node_.advertise<visualization_msgs::Marker>( Topics::kTestCubeTopic, 0 );
        }

        void drawTestingCube() const;

        void drawLidarData(const std::shared_ptr<DataLoader::LidarScanDataModel>) const;
        void drawImuData(const std::shared_ptr<DataLoader::ImuImuDataModel>) const;
        void drawRGBImage(const std::shared_ptr<DataLoader::CameraFrameDataModel>) const;
        void drawIRImage(const std::shared_ptr<DataLoader::CameraIrFrameDataModel>) const;
        void drawGnssPoseData(const std::shared_ptr<DataLoader::GnssPoseDataModel>) const;

    private:

        ros::NodeHandle& node_;
        LogService& logger_;

        ros::Publisher testCubePublisher_;

        LidarVisualizer lidarVisualizer_;
        ImuVisualizer imuVisualizer_;
        CameraVisualizer cameraVisualizer_;
        GnssVisualizer gnssVisualizer_;
        TFVisualizer tfTreeVisualizer_;


        visualization_msgs::Marker getTestCube() const;
    };

}
