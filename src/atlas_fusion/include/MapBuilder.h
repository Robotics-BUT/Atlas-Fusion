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
#include "algorithms/pointcloud/LaserSegmenter.h"
#include "algorithms/pointcloud/GlobalPointcloudStorage.h"
#include "algorithms/yolo_reprojection/YoloDetectionReprojector.h"
#include "algorithms/pointcloud/ObjectDetector.h"
#include "algorithms/image_processing/SimpleImageProcessor.h"

#include "data_loader/RecordingConstants.h"
#include "data_loader/DataLoader.h"
#include "data_writers/YoloDetectionWriter.h"
#include "data_writers/Lidar2ImagePlotter.h"

#include "visualizers/VisualizationHandler.h"

#include "Context.h"
#include "DataCache.h"

#include "local_map/LocalMap.h"
#include "local_map/ObjectsAggregator.h"

#include "data_models/lidar/LidarScanDataModel.h"

#include "fail_check/all.h"
#include "Timer.h"

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
                ros::NodeHandle &node,
                Context &context,
                const uint32_t gnssLogPoseNo,
                const uint32_t imuLogPoseNo,
                const float selfModelProcessNoise,
                const float selfModelObservationNoise,
                const float leafSize,
                const float globalLeafSize,
                const uint32_t noOfBatchesPerScan,
                const float lidarAggregationTime,
                const uint32_t noOfLasersPerLidar,
                const uint32_t noOfLaserAggregatedPoints,
                const DataLoader::timestamp_type keepHistoryLength,
                const double maxReplayerRate,
                const float maxLidar2ImgDist,
                const std::string &destinationFolder,
                const float vectorizer_sigma,
                const size_t segmenter_step,
                const float segmenter_lower_bound,
                const float segmenter_upper_bound,
                const float segmenter_scaling)
                : node_{node},
                  context_{context},
                  destinationFolder_{destinationFolder},
                  gnssPoseLogger_{context, gnssLogPoseNo},
                  imuPoseLogger_{context, imuLogPoseNo},
                  selfModel_{context, selfModelProcessNoise, selfModelObservationNoise},
                  detectionProcessor_{context},
                  pointCloudProcessor_{context, leafSize},
                  pointCloudExtrapolator_{context, pointCloudProcessor_, noOfBatchesPerScan},
                  pointCloudAggregator_{context, pointCloudProcessor_, lidarAggregationTime},
                  depthMap_{context, pointCloudProcessor_},
                  leftLidarLaserAggregator_{context, noOfLasersPerLidar, noOfLaserAggregatedPoints},
                  rightLidarLaserAggregator_{context, noOfLasersPerLidar, noOfLaserAggregatedPoints},
                  leftLaserSegmenter_{context, noOfLaserAggregatedPoints, vectorizer_sigma, segmenter_step,
                                      segmenter_lower_bound, segmenter_upper_bound, segmenter_scaling},
                  rightLaserSegmenter_{context, noOfLaserAggregatedPoints, vectorizer_sigma, segmenter_step,
                                       segmenter_lower_bound, segmenter_upper_bound, segmenter_scaling},
                  globalPointcloudStorage_{context, globalLeafSize},
                  occGrid_{context},
                  lidarObjectDetector_{context},
                  yoloIrReprojector_{context},
                  failChecker_{context},
                  visualizationHandler_{node, context},
                  dataLoader_{context, keepHistoryLength},
                  keepHistoryLength_{keepHistoryLength},
                  maxReplayerRate_{maxReplayerRate},
                  yoloIRDetectionWriter_{context, destinationFolder + DataLoader::Folders::kOutputFolder,
                                         DataLoader::Files::kIrCameraYoloFile},
                  lidarIrImgPlotter_{context, maxLidar2ImgDist, destinationFolder},
                  lidarRgbLFImgPlotter_{context, maxLidar2ImgDist, destinationFolder},
                  localMap_{context},
                  objectAggregator_{context} {
            lidarFilter_.enableFilterNearObjects();
        }

        /**
         *  Loads the common structured data from the folder defined in the MapBuilder constructor
         */
        void loadData();

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

        ros::NodeHandle &node_;
        Context &context_;
        std::string destinationFolder_;
        DataCache cache_;

        Algorithms::SimpleTrajectoryLogger gnssPoseLogger_;
        Algorithms::SimpleTrajectoryLogger imuPoseLogger_;
        Algorithms::SelfModel selfModel_;
        Algorithms::ImuDataProcessor imuProcessor_;
        Algorithms::LidarFilter lidarFilter_;
        Algorithms::DetectionsProcessor detectionProcessor_;

        Algorithms::PointCloudProcessor pointCloudProcessor_;
        Algorithms::PointCloudExtrapolator pointCloudExtrapolator_;
        Algorithms::PointCloudAggregator pointCloudAggregator_;

        Algorithms::DepthMap depthMap_;

        Algorithms::LaserAggregator leftLidarLaserAggregator_;
        Algorithms::LaserAggregator rightLidarLaserAggregator_;

        Algorithms::LaserSegmenter leftLaserSegmenter_;
        Algorithms::LaserSegmenter rightLaserSegmenter_;

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
        DataWriters::Lidar2ImagePlotter lidarRgbLFImgPlotter_;

        LocalMap::LocalMap localMap_;
        LocalMap::ObjectsAggregator objectAggregator_;

        Algorithms::SimpleImageProcessor simpleImageProcessor_;

        void processRGBCameraData(const std::shared_ptr<DataModels::CameraFrameDataModel> &, const std::string &);

        void processIRCameraData(const std::shared_ptr<DataModels::CameraIrFrameDataModel> &);

        void processGnssPoseData(const std::shared_ptr<DataModels::GnssPoseDataModel>&);

        void processImuDQuatData(const std::shared_ptr<DataModels::ImuDquatDataModel>&);

        void processImuGnssData(const std::shared_ptr<DataModels::ImuGnssDataModel>&);

        void processImuImuData(const std::shared_ptr<DataModels::ImuImuDataModel>&);

        void processLidarScanData(const std::shared_ptr<DataModels::LidarScanDataModel>&);

        void processRadarTiData(const std::shared_ptr<DataModels::RadarTiDataModel>&);

        void aggregateLidar(const std::shared_ptr<DataModels::LidarScanDataModel> &);

        void approximateLidar(const std::shared_ptr<DataModels::LidarScanDataModel> &);

        void generateDepthMapForIR() {};

        void projectRGBDetectionsToIR() {};
    };

}