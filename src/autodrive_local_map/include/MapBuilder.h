#pragma once

#include <ros/ros.h>

#include "algorithms/SimpleTrajectoryLogger.h"
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
#include "algorithms/yolo_reprojection/YoloDetectionReprojector.h"
#include "algorithms/pointcloud/ObjectDetector.h"

#include "data_loader/RecordingConstants.h"
#include "data_loader/DataLoader.h"
#include "data_writers/YoloDetectionWriter.h"
#include "data_writers/Lidar2ImagePlotter.h"

#include "visualizers/VisualizationHandler.h"

#include "Context.h"

#include "local_map/LocalMap.h"
#include "local_map/ObjectsAggregator.h"

#include "data_models/lidar/LidarScanDataModel.h"

#include "fail_check/all.h"

namespace AutoDrive {

    /**
     *  Map Builder is a container that is used for data and pipeline handling. The class provides simple interface
     *  for a data loading, starting data procession pipeline and memory clearance.
     */

    class MapBuilder {

    public:

        /**
         * Constructor defines the basic parameters of the map building pipeline by the arguments
         *  @param node ROS node
         *  @param context singleton that provides access to the global services
         *  @param gnssLogPoseNo pipeline will remember last N GNSS positions
         *  @param imuLogPoseNo pipeline will remember last N IMU positions
         *  @param selfModelProcessNoise process noise for the self-dynamic modeling kalman filter
         *  @param selfModelObservationNoise observation noise for the self-dynamic modeling kalman filter
         *  @param leafSize leaf size for the point cloud subsampling
         *  @param globalLeafSize leaf size for the global point cloud subsampling
         *  @param noOfBatchesPerScan number of batches that single lidar scan will be devided into
         *  @param liadrAggregationTime the time for which the lidar scans will be holded
         *  @param noOfLasersPerLidar number of lasers in the lidar scan
         *  @param noOfLaserAggregatedPoints number of points aggregated by the single laser aggregator
         *  @param keepHistoryLength how long should data loaders keep reference to the old data
         *  @param maxReplayerRate maximum ratio, how do offline data could be replayed
         *  @param maxLidar2ImgDist maximum distance in which the lidar data will be plotted into the image frame
         *  @param destinationFolder folder where the output data will be writen
         */
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
                double maxReplayerRate,
                float maxLidar2ImgDist,
                std::string destinationFolder)
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
        , yoloIrReprojector_{context}
        , lidarObjectDetector_{context}
        , failChecker_{context}
        , visualizationHandler_{node, context}
        , dataLoader_{context, keepHistoryLength}
        , keepHistoryLength_{keepHistoryLength}
        , maxReplayerRate_{maxReplayerRate}
        , yoloIRDetectionWriter_{context, destinationFolder + DataLoader::Folders::kYoloFolder, DataLoader::Files::kIrCameraYoloFile}
        , lidarIrImgPlotter_{context, maxLidar2ImgDist, destinationFolder}
        , localMap_{context}
        , objectAggregator_{context}
        {

            lidarFilter_.enableFilterNearObjects();
        }

        /**
         *  Loads the common structured data from the folder defined by the input argument
         *  @param path system path to the folder, where offline data are stored
         */
        void loadData(const std::string& path);

        /**
         *  Method runs the main data processing pipeline. Map Builder class reads raw data one by one and creates the
         *  map of the surrounding in this way
         */
        void buildMap();

        /**
         *  Clears the Map Builder memory
         */
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

        Algorithms::YoloDetectionReprojector yoloIrReprojector_;

        FailCheck::FailChecker failChecker_;

        Visualizers::VisualizationHandler visualizationHandler_;

        DataLoader::DataLoader dataLoader_;
        DataLoader::timestamp_type keepHistoryLength_;
        double maxReplayerRate_;

        DataWriters::YoloDetectionWriter yoloIRDetectionWriter_;
        DataWriters::Lidar2ImagePlotter lidarIrImgPlotter_;

        LocalMap::LocalMap localMap_;
        LocalMap::ObjectsAggregator objectAggregator_;

        std::map<std::string, std::shared_ptr<DataModels::LidarScanDataModel>> lidarDataHistory_;

        std::string getFrameForData(std::shared_ptr<DataModels::GenericDataModel>);
    };

}