#pragma once

#include <rtl/Quaternion.h>

rtl::Quaternion<double> rpyToQuaternion(double roll, double pitch, double yaw) {

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    rtl::Quaternion<double> q;
    q.setW(cy * cp * cr + sy * sp * sr);
    q.setX(cy * cp * sr - sy * sp * cr);
    q.setY(sy * cp * sr + cy * sp * cr);
    q.setZ(sy * cp * cr - cy * sp * sr);

    return q;
}


void quaternionToRPY(rtl::Quaternion<double> q, double& roll, double &pitch, double& yaw) {

    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    yaw = std::atan2(siny_cosp, cosy_cosp);
}