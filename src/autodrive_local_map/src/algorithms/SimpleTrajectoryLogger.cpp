#include "algorithms/SimpleTrajectoryLogger.h"
#include "local_map/Frames.h"

namespace AutoDrive::Algorithms {



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
                                                    {},
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

        rtl::Quaternion<double> q;
        q.setW(cy * cp * cr + sy * sp * sr);
        q.setX(cy * cp * sr - sy * sp * cr);
        q.setY(sy * cp * sr + cy * sp * cr);
        q.setZ(sy * cp * cr - cy * sp * sr);

        return q;
    }
}