#include <iostream>
#include <sstream>
#include <opencv/highgui.h>
#include <ros/ros.h>

#include <QString>
#include <QDebug>
#include "MapBuilder.h"

#include <rtl/Vector3D.h>

#include "ConfigService.h"
#include "LogService.h"

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

int main(int argc, char** argv) {

//    std::cout << "Hello" << std::endl;
//
//    cv::Mat mat(3,3,CV_32F);
//    std::cout << mat << std::endl;
//
//    QString str = QString("QHello");
//    qDebug() << str;
//    ros::NodeHandle node;
//    ros::spin();

    if(argc < 2) {
        std::cerr << "Error: too few input arguments!" << std::endl;
        std::cerr << " Usage: autodrive_local_map <path_to_config_file>" << std::endl;
    }

    AutoDrive::ConfigService configService(argv[1]);


    auto log_file = configService.getStringValue({"logger", "log_file"});
    auto log_ext = configService.getStringValue({"logger", "log_ext"});
    auto log_lvl = static_cast<AutoDrive::LogService::LogLevel>(configService.getUInt32Value({"logger", "log_lvl"}));
    std::stringstream ss;
    ss << log_file << "_" << currentDateTime() << "." << log_ext;
    auto logger = AutoDrive::LogService(ss.str(), log_lvl, true, true);
    if (!logger.openFile()) {
        std::cerr << "Unable to start logging" << std::endl;
        return 1;
    }

    logger.info("Initialization Done!");

    AutoDrive::MapBuilder mapBuilder{};
    mapBuilder.buildMap();

    rtl::Vector3D<double> vector;



    return 0;
}
