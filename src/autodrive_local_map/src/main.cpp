#include <iostream>
#include <sstream>
#include <opencv/highgui.h>
#include <ros/ros.h>

#include <QString>
#include <QDebug>
#include "MapBuilder.h"

#include <rtl/Core.h>
#include <rtl/Transformation.h>

#include "ConfigService.h"
#include "Context.h"
#include "visualizers/TFVisualizer.h"
#include "local_map/Frames.h"
#include "munkres/munkres.h"

rtl::RigidTf3D<double> getTFFrameFromConfig(AutoDrive::ConfigService& service, std::string name) {
    auto translation = service.getVector3DValue<double>({name, "trans"});
    auto rotation = service.getQuaternionValue<double>({name, "rot"});
    rtl::RigidTf3D<double> frame{ rotation, translation};
    return frame;
}


AutoDrive::LocalMap::TFTree buildTFTree(std::string rootFrame, std::vector<std::string> frames, std::string tfFilePath, AutoDrive::LogService& logger) {

    AutoDrive::ConfigService TFConfigService(std::move(tfFilePath));
    AutoDrive::LocalMap::TFTree tfTree(rootFrame, logger);
    for(const auto& frameName : frames) {
        auto frame = getTFFrameFromConfig(TFConfigService, frameName);
        tfTree.addFrame(frame, frameName);
    }
    return tfTree;
}


int main(int argc, char** argv) {


    int cols =3;
    int rows = 4;
    std::vector<int> M = {2, 2, 1,
                          1, 2, 2,
                          2, 1, 2,
                          2, 2, 2};
    auto f = [&](unsigned r, unsigned c) { return M[r * cols + c]; };
    auto matching = Munkres::munkres_algorithm<int>(rows, cols, f);

    const std::string imuFrameName = "imu";

    if(argc < 2) {
        std::cerr << "Error: too few input arguments!" << std::endl;
        std::cerr << " Usage: autodrive_local_map <path_to_config_file>" << std::endl;
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

    std::string rootFrame = AutoDrive::LocalMap::Frames::kImuFrame;
    std::vector<std::string> childFrames = {AutoDrive::LocalMap::Frames::kGnssAntennaFront,
                                            AutoDrive::LocalMap::Frames::kGnssAntennaRear,
                                            AutoDrive::LocalMap::Frames::kLidarLeft,
                                            AutoDrive::LocalMap::Frames::kLidarRight,
                                            AutoDrive::LocalMap::Frames::kCameraLeftFront,
                                            AutoDrive::LocalMap::Frames::kCameraLeftSide,
                                            AutoDrive::LocalMap::Frames::kCameraRightFront,
                                            AutoDrive::LocalMap::Frames::kCameraRightSide,
                                            AutoDrive::LocalMap::Frames::kCameraIr};
    auto calibFolder = configService.getStringValue({"calibratios_folder"});
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
    auto aggretationTime = configService.getFloatValue({"lidar_aggregator", "aggretation_time"});

    auto selfModelProcessNoise = configService.getFloatValue({"self_model", "kalman_process_noise"});
    auto selfModelObservationNoise = configService.getFloatValue({"self_model", "kalman_observation_noise"});

    auto gnssLogNo = configService.getUInt32Value({"pose_logger", "gnss"});
    auto imuLogNo = configService.getUInt32Value({"pose_logger", "imu"});

    auto keepHistorySecLength = static_cast<AutoDrive::DataLoader::timestamp_type>(configService.getDoubleValue({"map_builder", "keep_history_sec_length"}) * 1e9); // to nanoseconds
    auto dataFolder = configService.getStringValue({"data_folder"});
    auto maxReplayerRate = configService.getDoubleValue({"map_builder", "max_replayer_rate"});

    auto lidarPlotterMaxDist = configService.getFloatValue({"lidar_img_plotter", "max_distance"});

    auto context = AutoDrive::Context(logger, tfTree, calibFolder);

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
        aggretationTime,
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

    mapBuilder.loadData(dataFolder);
    mapBuilder.buildMap();
    mapBuilder.clearData();

    return 0;
}
