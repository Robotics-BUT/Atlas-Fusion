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

#include "algorithms/SimpleTrajectoryLogger.h"
#include "local_map/Frames.h"

namespace AtlasFusion::Algorithms {



    void SimpleTrajectoryLogger::onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data) {

        auto gnssPose = DataModels::GlobalPosition{
                data->getLatitude(),
                data->getLongitude(),
                data->getAltitude(),
                data->getAzimut()};
        auto imuPose = gnssPoseToRootFrame(gnssPose);

        if(!initialized_) {
            initPositionGnss_ = gnssPoseToRootFrame(gnssPose);
            initialized_ = true;
        }

        while(positionHistory_.size() >= historyLenght_) {
            positionHistory_.pop_front();
        }

        auto offset = DataModels::GlobalPosition::getOffsetBetweenCoords(initPositionGnss_, imuPose);
        position_.setPositon(offset);

        auto azimut = data->getAzimut();
        position_.setOrientation(rpyToQuaternion(0,0,-azimut*M_PI/180));

        positionHistory_.push_back(position_);
    }


    void SimpleTrajectoryLogger::onImuGps(std::shared_ptr<DataModels::ImuGnssDataModel> data) {
        auto imuPose = DataModels::GlobalPosition{
                data->getLatitude(),
                data->getLongitude(),
                data->getAltitude(),
                0};
        //auto imuPose = gnssPoseToRootFrame(gnssPose);

        if(!initialized_) {
            initPositionGnss_ = imuPose;
            initialized_ = true;
        }

        while(positionHistory_.size() >= historyLenght_) {
            positionHistory_.pop_front();
        }

        double altitudeTemp = position_.getPosition().z();
        auto offset = DataModels::GlobalPosition::getOffsetBetweenCoords(initPositionGnss_, imuPose);
        position_.setPositon(offset);
        position_.setZ(altitudeTemp);

        positionHistory_.push_back(position_);
    }


    DataModels::GlobalPosition SimpleTrajectoryLogger::gnssPoseToRootFrame(const DataModels::GlobalPosition gnssPose) {

        auto gnssOffset = DataModels::LocalPosition{context_.tfTree_.transformPointFromFrameToFrame({}, LocalMap::Frames::kGnssAntennaRear, LocalMap::Frames::kImuFrame),
                                                    rtl::Quaternion<double>::identity(),
                                                    0};
        return DataModels::GlobalPosition::localPoseToGlobalPose(gnssOffset, gnssPose);
    }


    rtl::Quaternion<double> SimpleTrajectoryLogger::rpyToQuaternion(double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
    {
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        rtl::Quaternion<double> q = rtl::Quaternion<double>::identity();
        q.setW(cy * cp * cr + sy * sp * sr);
        q.setX(cy * cp * sr - sy * sp * cr);
        q.setY(sy * cp * sr + cy * sp * cr);
        q.setZ(sy * cp * cr - cy * sp * sr);

        return q;
    }
}