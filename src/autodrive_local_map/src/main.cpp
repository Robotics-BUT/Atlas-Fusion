#include <iostream>
#include <opencv/highgui.h>
#include <ros/ros.h>

#include <QString>
#include <QDebug>
#include "MapBuilder.h"

#include <robotic-template-library/Vector3D.h>

int main(int argc, char** argv) {

    std::cout << "Hello" << std::endl;

    cv::Mat mat(3,3,CV_32F);
    std::cout << mat << std::endl;

    QString str = QString("QHello");
    qDebug() << str;

//    ros::NodeHandle node;
//    ros::spin();

    AutoDrive::MapBuilder mapBuilder{};
    mapBuilder.buildMap();

    Vector3D vector;



    return 0;
}
