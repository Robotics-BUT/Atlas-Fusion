#pragma once

#include <ros/ros.h>

#include "algorithms/SimpleTrajectoryLogger.h"
#include "algorithms/MovementModel.h"
#include "algorithms/ImuDataProcessor.h"
#include "algorithms/LidarFilter.h"
#include "algorithms/DepthMap.h"
#include "algorithms/DetectionsProcessor.h"
#include "algorithms/pointcloud/PointCloudExtrapolator.h"
#include "algorithms/pointcloud/PointCloudAggregator.h"
#include "algorithms/pointcloud/OccupancyGrid3D.h"


#include "data_loader/DataLoader.h"
#include "visualizers/VisualizationHandler.h"

#include "Context.h"

#include "local_map/LocalMap.h"
#include "data_models/lidar/LidarScanDataModel.h"

namespace AutoDrive {

    class MapBuilder {

    public:

        explicit MapBuilder(ros::NodeHandle& node, Context& context, DataLoader::timestamp_type keepHistoryLength, double maxReplayerRate)
        : node_{node}
        , context_{context}
        , gnssPoseLogger_{context, 100}
        , imuPoseLogger_{context, 1000}
        , selfModel_{context, 1, 1}
        , depthMap_{context}
        , detectionProcessor_{context}
        , pointCloudExtrapolator_{context, 10}
        , pointCloudAggregator_{context, 1.0}
        , occGrid_{context}
        , visualizationHandler_(node, context)
        , dataLoader_(context, keepHistoryLength)
        , keepHistoryLength_(keepHistoryLength)
        , maxReplayerRate_(maxReplayerRate)
        , localMap_{context}
        {

            lidarFilter_.enableFilterNearObjects();
        }

        void loadData(const std::string&);
        void buildMap();
        void clearData();

    private:

        ros::NodeHandle& node_;
        Context& context_;

        Algorithms::SimpleTrajectoryLogger gnssPoseLogger_;
        Algorithms::SimpleTrajectoryLogger imuPoseLogger_;
        Algorithms::MovementModel selfModel_;
        Algorithms::ImuDataProcessor imuProcessor_;
        Algorithms::LidarFilter lidarFilter_;
        Algorithms::DepthMap depthMap_;
        Algorithms::DetectionsProcessor detectionProcessor_;

        Algorithms::PointCloudExtrapolator pointCloudExtrapolator_;
        Algorithms::PointCloudAggregator pointCloudAggregator_;
        Algorithms::OccupancyGrid3D occGrid_;

        Visualizers::VisualizationHandler visualizationHandler_;

        DataLoader::DataLoader dataLoader_;
        DataLoader::timestamp_type keepHistoryLength_;
        double maxReplayerRate_;

        LocalMap::LocalMap localMap_;

        std::map<std::string, std::shared_ptr<DataModels::LidarScanDataModel>> lidarDataHistory_;

        [[deprecated]]
        rtl::Transformation3D<double> getCameraTf(const DataLoader::CameraIndentifier&);
        std::string getCameraFrame(const DataLoader::CameraIndentifier& id);
        std::string getLidarFrame(const DataLoader::LidarIdentifier & id);

    };

}