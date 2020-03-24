#pragma once

#include <ros/ros.h>

#include "algorithms/SimpleTrajectoryLogger.h"
#include "algorithms/MovementModel.h"
#include "algorithms/SelfModel.h"
#include "algorithms/ImuDataProcessor.h"
#include "algorithms/LidarFilter.h"
#include "algorithms/DepthMap.h"
#include "algorithms/DetectionsProcessor.h"
#include "algorithms/pointcloud/PointCloudExtrapolator.h"
#include "algorithms/pointcloud/PointCloudAggregator.h"
#include "algorithms/pointcloud/PointCloudProcessor.h"
#include "algorithms/pointcloud/OccupancyGrid3D.h"
#include "algorithms/pointcloud/LaserAggregator.h"
#include "algorithms/pointcloud/GlobalPointcloudStorage.h"
#include "algorithms/pointcloud/ObjectDetector.h"


#include "data_loader/DataLoader.h"
#include "visualizers/VisualizationHandler.h"

#include "Context.h"

#include "local_map/LocalMap.h"
#include "local_map/ObjectsAggregator.h"

#include "data_models/lidar/LidarScanDataModel.h"

#include "fail_check/all.h"

namespace AutoDrive {

    class MapBuilder {

    public:

        explicit MapBuilder(
                ros::NodeHandle& node,
                Context& context,
                uint32_t gnssLogPoseNo,
                uint32_t imuLogPoseNo,
                float selfModelProcessNoise,
                float selfModelObservationNoise,
                float leafSize,
                float globalLeafSize,
                uint32_t noOfBatchesPerScan,
                float liadrAggregationTime,
                uint32_t noOfLasersPerLidar,
                uint32_t noOfLaserAggregatedPoints,
                DataLoader::timestamp_type keepHistoryLength,
                double maxReplayerRate)
        : node_{node}
        , context_{context}
        , gnssPoseLogger_{context, gnssLogPoseNo}
        , imuPoseLogger_{context, imuLogPoseNo}
        , selfModel_{context, selfModelProcessNoise, selfModelObservationNoise}
        , depthMap_{context}
        , detectionProcessor_{context}
        , pointCloudExtrapolator_{context, noOfBatchesPerScan}
        , pointCloudAggregator_{context, liadrAggregationTime}
        , pointCloudProcessor_ {context, leafSize}
        , leftLidarLaserAggregator_{context, noOfLasersPerLidar, noOfLaserAggregatedPoints}
        , rightLidarLaserAggregator_{context, noOfLasersPerLidar, noOfLaserAggregatedPoints}
        , globalPointcloudStorage_{context, globalLeafSize}
        , occGrid_{context}
        , lidarObjectDetector_{context}
        , failChecker_{context}
        , visualizationHandler_(node, context)
        , dataLoader_(context, keepHistoryLength)
        , keepHistoryLength_(keepHistoryLength)
        , maxReplayerRate_(maxReplayerRate)
        , localMap_{context}
        , objectAggregator_{context}
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
        Algorithms::SelfModel selfModel_;
        Algorithms::ImuDataProcessor imuProcessor_;
        Algorithms::LidarFilter lidarFilter_;
        Algorithms::DepthMap depthMap_;
        Algorithms::DetectionsProcessor detectionProcessor_;

        Algorithms::PointCloudExtrapolator pointCloudExtrapolator_;
        Algorithms::PointCloudAggregator pointCloudAggregator_;
        Algorithms::PointCloudProcessor pointCloudProcessor_;
        Algorithms::LaserAggregator leftLidarLaserAggregator_;
        Algorithms::LaserAggregator rightLidarLaserAggregator_;
        Algorithms::GlobalPointcloudStorage globalPointcloudStorage_;
        Algorithms::OccupancyGrid3D occGrid_;
        Algorithms::ObjectDetector lidarObjectDetector_;

        FailCheck::FailChecker failChecker_;

        Visualizers::VisualizationHandler visualizationHandler_;

        DataLoader::DataLoader dataLoader_;
        DataLoader::timestamp_type keepHistoryLength_;
        double maxReplayerRate_;

        LocalMap::LocalMap localMap_;
        LocalMap::ObjectsAggregator objectAggregator_;

        std::map<std::string, std::shared_ptr<DataModels::LidarScanDataModel>> lidarDataHistory_;

        std::string getFrameForData(std::shared_ptr<DataModels::GenericDataModel>);
    };

}