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

#include <iostream>
#include <sstream>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <QString>
#include <QDebug>
#include <utility>
#include "MapBuilder.h"

#include <rtl/Transformation.h>

#include "ConfigService.h"
#include "Context.h"
#include "munkres/munkres.h"

rtl::RigidTf3D<double> getTFFrameFromConfig(AutoDrive::ConfigService &service, const AutoDrive::FrameType &type) {
    auto translation = service.getVector3DValue<double>({frameTypeName(type), "trans"});
    auto rotation = service.getQuaternionValue<double>({frameTypeName(type), "rot"});
    rtl::RigidTf3D<double> frame{rotation, translation};
    return frame;
}


AutoDrive::LocalMap::TFTree buildTFTree(AutoDrive::FrameType rootFrame, const std::vector<AutoDrive::FrameType> &frames, std::string tfFilePath, AutoDrive::LogService &logger) {
    AutoDrive::ConfigService TFConfigService(std::move(tfFilePath));
    AutoDrive::LocalMap::TFTree tfTree(rootFrame, logger);
    for (const auto &frameType: frames) {
        auto frame = getTFFrameFromConfig(TFConfigService, frameType);
        tfTree.addFrame(frame, frameType);
    }
    return tfTree;
}

AutoDrive::FunctionalityFlags loadFunctionalityFlags(AutoDrive::ConfigService &confService) {
    AutoDrive::FunctionalityFlags ffEntity(
            confService.getBoolValue({"functionalities", "generate_depth_map_for_ir"}),
            confService.getBoolValue({"functionalities", "rgb_to_ir_detection_projection"}),
            confService.getBoolValue({"functionalities", "short_term_lidar_aggregation"}),
            confService.getBoolValue({"functionalities", "lidar_laser_approximations_and_segmentation"}),
            confService.getBoolValue({"functionalities", "global_lidar_aggregation"}),
            confService.getBoolValue({"visualizations", "visualization_global_enable"}),
            confService.getBoolValue({"visualizations", "rgb_camera_visualization"}),
            confService.getBoolValue({"visualizations", "ir_camera_visualization"}),
            confService.getBoolValue({"visualizations", "lidar_visualization"}),
            confService.getBoolValue({"visualizations", "imu_visualization"}),
            confService.getBoolValue({"visualizations", "gnss_visualization"}),
            confService.getBoolValue({"visualizations", "radar_visualization"}));
    return ffEntity;
}


int main(int argc, char **argv) {

    if (argc < 2) {
        std::cerr << "Error: too few input arguments!" << std::endl;
        std::cerr << " Usage: atlas_fusion <path_to_config_file>" << std::endl;
    }

    AutoDrive::ConfigService configService(argv[1]);
    auto log_file = configService.getStringValue({"logger", "log_file"});
    auto log_ext = configService.getStringValue({"logger", "log_ext"});
    auto log_lvl = static_cast<AutoDrive::LogService::LogLevel>(configService.getUInt32Value({"logger", "log_lvl"}));

    std::stringstream ss;
    ss << log_file << "_" << AutoDrive::LogService::currentDateTime() << "." << log_ext;
    auto logger = AutoDrive::LogService(ss.str(), log_lvl, true, true);
    if (!logger.openFile()) {
        std::cerr << "Unable to start logging" << std::endl;
        return 1;
    }
    logger.info("Initialization Done!");

    ros::init(argc, argv, "autodrive_localmap");
    ros::NodeHandle node{};

    AutoDrive::FrameType rootFrame = AutoDrive::FrameType::kImu;
    std::vector<AutoDrive::FrameType> childFrames = {AutoDrive::FrameType::kGnssAntennaFront,
                                            AutoDrive::FrameType::kGnssAntennaRear,
                                            AutoDrive::FrameType::kLidarLeft,
                                            AutoDrive::FrameType::kLidarRight,
                                            AutoDrive::FrameType::kLidarCenter,
                                            AutoDrive::FrameType::kRadarTi,
                                            AutoDrive::FrameType::kCameraLeftFront,
                                            AutoDrive::FrameType::kCameraLeftSide,
                                            AutoDrive::FrameType::kCameraRightFront,
                                            AutoDrive::FrameType::kCameraRightSide,
                                            AutoDrive::FrameType::kCameraIr
    };
    auto calibFolder = configService.getStringValue({"calibrations_folder"});
    auto tfTree = buildTFTree(
            rootFrame,
            childFrames,
            std::string(calibFolder + "frames.yaml"),
            logger);

    auto lasersPerLidar = configService.getUInt32Value({"laser_aggregator", "lasers"});
    auto pointsPerLaser = configService.getUInt32Value({"laser_aggregator", "points_per_laser"});

    auto leafSize = configService.getFloatValue({"lidar_aggregator", "leaf_size"});
    auto globalLeafSize = configService.getFloatValue({"lidar_aggregator", "global_leaf_size"});
    auto batchesPerScan = configService.getUInt32Value({"lidar_aggregator", "no_of_batches_per_scan"});
    auto aggregationTime = configService.getFloatValue({"lidar_aggregator", "aggregation_time"});

    auto selfModelProcessNoise = configService.getFloatValue({"self_model", "kalman_process_noise"});
    auto selfModelObservationNoise = configService.getFloatValue({"self_model", "kalman_observation_noise"});

    auto gnssLogNo = configService.getUInt32Value({"pose_logger", "gnss"});
    auto imuLogNo = configService.getUInt32Value({"pose_logger", "imu"});

    auto keepHistorySecLength = static_cast<AutoDrive::DataLoader::timestamp_type>(configService.getDoubleValue({"map_builder", "keep_history_sec_length"}) *
                                                                                   1e9); // to nanoseconds
    auto dataFolder = configService.getStringValue({"data_folder"});
    auto maxReplayerRate = configService.getDoubleValue({"map_builder", "max_replayer_rate"});

    auto lidarPlotterMaxDist = configService.getFloatValue({"lidar_img_plotter", "max_distance"});

    auto functionalityFlags = loadFunctionalityFlags(configService);
    auto context = AutoDrive::Context(logger, tfTree, calibFolder, functionalityFlags);

    auto vectorizer_sigma = configService.getFloatValue({"laser_segmenter", "vectorizer_sigma"});
    auto segmenter_step = configService.getUInt32Value({"laser_segmenter", "segmenter_step"});
    auto segmenter_lower_bound = configService.getFloatValue({"laser_segmenter", "segmenter_lower_bound"});
    auto segmenter_upper_bound = configService.getFloatValue({"laser_segmenter", "segmenter_upper_bound"});
    auto segmenter_scaling = configService.getFloatValue({"laser_segmenter", "segmenter_scaling"});

    AutoDrive::MapBuilder mapBuilder{
            node,
            context,
            gnssLogNo,
            imuLogNo,
            selfModelProcessNoise,
            selfModelObservationNoise,
            leafSize,
            globalLeafSize,
            batchesPerScan,
            aggregationTime,
            lasersPerLidar,
            pointsPerLaser,
            keepHistorySecLength,
            maxReplayerRate,
            lidarPlotterMaxDist,
            dataFolder,
            vectorizer_sigma,
            segmenter_step,
            segmenter_lower_bound,
            segmenter_upper_bound,
            segmenter_scaling};

    mapBuilder.loadData();
    mapBuilder.buildMap();
    mapBuilder.clearData();

    return 0;
}
