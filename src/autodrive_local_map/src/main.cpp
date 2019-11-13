#include <iostream>
#include <sstream>
#include <opencv/highgui.h>
#include <ros/ros.h>

#include <QString>
#include <QDebug>
#include "MapBuilder.h"

#include <rtl/Vector3D.h>
#include <rtl/Transformation3D.h>

#include "ConfigService.h"
#include "LogService.h"
#include "visualizers/TFVisualizer.h"
#include "TFTree.h"

rtl::Transformation3Dd getTFFrameFromConfig(AutoDrive::ConfigService& service, std::string name) {
    auto translation = service.getVector3DValue<double>({name, "trans"});
    auto rotation = service.getQuaternionValue<double>({name, "rot"});
    rtl::Transformation3Dd frame{translation, rotation};
    return frame;
}


AutoDrive::TFTree buildTFTree(std::string&& rootFrame, std::vector<std::string>&& frames, std::string tfFilePath, AutoDrive::LogService& logger) {

    AutoDrive::ConfigService TFConfigService(std::move(tfFilePath));
    AutoDrive::TFTree tfTree(rootFrame, logger);
    for(const auto& frameName : frames) {
        auto frame = getTFFrameFromConfig(TFConfigService, frameName);
        tfTree.addFrame(frame, frameName);
    }
    return tfTree;
}


int main(int argc, char** argv) {

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

    auto tfTree = buildTFTree(
            "imu",
            {"gnss_front",
             "gnss_rear",
             "lidar_left",
             "lidar_right",
             "camera_left_front",
             "camera_left_side",
             "camera_right_front",
             "camera_right_side",
             "ir_camera"},
             std::string(configService.getStringValue({"tf_file"})),
             logger);

    auto keepHistorySecLength = static_cast<AutoDrive::DataLoader::timestamp_type>(configService.getDoubleValue({"map_builder", "keep_history_sec_length"}) * 1e9); // to nanoseconds
    auto dataFolder = configService.getStringValue({"data_folder"});
    auto maxReplayerRate = configService.getDoubleValue({"map_builder", "max_replayer_rate"});

    AutoDrive::MapBuilder mapBuilder{node, tfTree, logger, keepHistorySecLength, maxReplayerRate};
    mapBuilder.loadData(dataFolder);
    mapBuilder.buildMap();
    mapBuilder.clearData();

    return 0;
}
