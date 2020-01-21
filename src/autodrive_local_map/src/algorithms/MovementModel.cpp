#include "algorithms/MovementModel.h"
#include "local_map/Frames.h"

namespace AutoDrive::Algorithms {



    void MovementModel::onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data) {

        currentTime = data->getTimestamp();

        auto gnssPose = DataModels::GlobalPosition{
                data->getLatitude(),
                data->getLongitude(),
                data->getAltitude(),
                data->getAzimut()};
        auto imuPose = gnssPoseToRootFrame(gnssPose);
        auto yaw = -data->getAzimut() * M_PI / 180;
        gnssYaw_ = yaw;

        if(!initialized_) {
            initPositionGnss_ = gnssPoseToRootFrame(gnssPose);
            lastImuTimestamp_ = data->getTimestamp();
            lastDquatTimestamp_ = data->getTimestamp();
//            orientationOffset_ =  orientation_ * rpyToQuaternion(0,0,yaw).inverted() ;
            initialized_ = true;
        }

        while(positionHistory_.size() >= 100) {
            positionHistory_.pop_front();
        }


        auto offset = DataModels::GlobalPosition::getOffsetBetweenCoords(initPositionGnss_, imuPose);

        cv::Mat measurementX = (cv::Mat_<double>(2, 1) << offset.x(), 0);
        cv::Mat measurementY = (cv::Mat_<double>(2, 1) << offset.y(), 0);
        cv::Mat measurementZ = (cv::Mat_<double>(2, 1) << offset.z(), 0);
        kalmanX_.correct(measurementX);
        kalmanY_.correct(measurementY);
        kalmanZ_.correct(measurementZ);

        double roll, pitch, y;
        quaternionToRPY(orientation_, roll, pitch, y);

        DataModels::LocalPosition position{{kalmanX_.getPosition(), kalmanY_.getPosition(), kalmanZ_.getPosition()},
                                           rpyToQuaternion(roll,pitch, estimateHeading()),
//                                           orientation_ * orientationOffset_,
                                           data->getTimestamp()};

        positionHistory_.push_back(position);

        //std::cout << position.toString() << std::endl;
    }


    void MovementModel::onImuImuData(std::shared_ptr<DataModels::ImuImuDataModel> data) {

        currentTime = data->getTimestamp();

        if(initialized_) {

            auto imuToOriginTf = context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kImuFrame);
            imuToOriginTf.setTranslation({});

            imuProcessor_.setOrientation(data->getOrientation());
            auto linAccNoGrav = imuProcessor_.removeGravitaionAcceleration(data->getLinearAcc());

            orientation_ = data->getOrientation();
            double roll, pitch, yaw;
            quaternionToRPY(orientation_, roll, pitch, yaw);

            yaw = estimateHeading();
            auto accX = cos(yaw)*linAccNoGrav.x() - sin(yaw)*linAccNoGrav.y();
            auto accY = sin(yaw)*linAccNoGrav.x() + cos(yaw)*linAccNoGrav.y();

            auto dt = (data->getTimestamp() - lastImuTimestamp_) * 1e-9;
            kalmanX_.predict(dt, accX);
            kalmanY_.predict(dt, accY);
            kalmanZ_.predict(dt, linAccNoGrav.z());

            //std::cout << orientation_.x() << " " << orientation_.y()  << " " << orientation_.z()  << " " << orientation_.w() << std::endl;

            lastImuTimestamp_ = data->getTimestamp();
        }
    }


    void MovementModel::onImuDquatData(std::shared_ptr<DataModels::ImuDquatDataModel> data) {

        currentTime = data->getTimestamp();

        if(initialized_) {

//            auto dt = (data->getTimestamp() - lastImuTimestamp_) * 1e-9;
//            auto dQuat = data->getDQuat();
//
//            double roll, pitch, yaw;
//            quaternionToRPY(dQuat, roll, pitch, yaw);
//            kalmanYaw_.predict(dt, yaw);
//
            lastDquatTimestamp_ = data->getTimestamp();
        }
    }

    DataModels::LocalPosition MovementModel::getPosition() {

//        double heading = estimateHeading();
//        return DataModels::LocalPosition(
//                {kalmanX_.getPosition(), kalmanY_.getPosition(), kalmanZ_.getPosition()},
//                rpyToQuaternion(roll_, pitch_, heading),
//                currentTime);
        if(positionHistory_.empty()){
            return DataModels::LocalPosition{{},{},0};
        }
        return positionHistory_.back();
    }


    DataModels::LocalPosition MovementModel::estimatePositionInTime(const uint64_t time) { // currently returns the last position before the reference time

        for(int i = positionHistory_.size()-1; i >= 0  ; i--) {
            if(positionHistory_.at(i).getTimestamp() < time) {

                if(i == 0) {
                    return positionHistory_.at(i);
                } else {

                    auto poseBefore = &positionHistory_.at(std::min(static_cast<size_t>(i), positionHistory_.size()-2));
                    auto poseAfter = &positionHistory_.at(std::min(static_cast<size_t>(i+1), positionHistory_.size()-1));

                    auto numerator = static_cast<float>(poseAfter->getTimestamp() - time);
                    auto denominator = static_cast<float>(poseAfter->getTimestamp() - poseBefore->getTimestamp());
                    float ratio = numerator / denominator;

                    rtl::Vector3D<double> interpolatedPose{
                            poseBefore->getPosition().x() + (poseAfter->getPosition().x() - poseBefore->getPosition().x()) * ratio,
                            poseBefore->getPosition().y() + (poseAfter->getPosition().y() - poseBefore->getPosition().y()) * ratio,
                            poseBefore->getPosition().z() + (poseAfter->getPosition().z() - poseBefore->getPosition().z()) * ratio,
                    };

                    auto newOrientation = poseBefore->getOrientation().slerp(poseAfter->getOrientation(), ratio);

                    uint64_t duration = poseAfter->getTimestamp() - poseBefore->getTimestamp();
                    uint64_t ts = poseBefore->getTimestamp() + static_cast<uint64_t>(duration * ratio);

                    return DataModels::LocalPosition{interpolatedPose, newOrientation, ts};
                }
            }
        }
        context_.logger_.warning("Unable to estimate position in time! Missing time point in history.");
        return DataModels::LocalPosition{{},{},0};
    }

    DataModels::LocalPosition MovementModel::interpolateLocalPosition( DataModels::LocalPosition& begin, DataModels::LocalPosition& end, float ratio) {

        rtl::Vector3D<double> interpolatedPose{
                begin.getPosition().x() + (end.getPosition().x() - begin.getPosition().x()) * ratio,
                begin.getPosition().y() + (end.getPosition().y() - begin.getPosition().y()) * ratio,
                begin.getPosition().z() + (end.getPosition().z() - begin.getPosition().z()) * ratio,
        };

        auto newOrientation = begin.getOrientation().slerp(end.getOrientation(), ratio);

        uint64_t duration = end.getTimestamp() - begin.getTimestamp();
        uint64_t time = begin.getTimestamp() + static_cast<uint64_t>(duration * ratio);

        return DataModels::LocalPosition{interpolatedPose, newOrientation, time};
    }


    double MovementModel::estimateHeading() {

        return gnssYaw_;

        auto speedHeading = atan2(kalmanY_.getSpeed(), kalmanX_.getSpeed());
        auto speedQuaternion = rtl::Quaternion<double>{0, 0, speedHeading};
        auto gnssQuaternion = rtl::Quaternion<double>{0, 0, gnssYaw_};

        auto speed = std::sqrt( std::pow(kalmanY_.getSpeed(),2) + std::pow(kalmanX_.getSpeed(),2));
        auto gain = getSpeedGainFactor(speed);

        double r,p,y;
        auto result = speedQuaternion.slerp(gnssQuaternion, gain);
        quaternionToRPY(result, r, p , y);

        return y;
    }

    DataModels::GlobalPosition MovementModel::gnssPoseToRootFrame(const DataModels::GlobalPosition gnssPose) {

        auto gnssOffset = DataModels::LocalPosition{context_.tfTree_.transformPointFromFrameToFrame({}, LocalMap::Frames::kGnssAntennaRear, LocalMap::Frames::kImuFrame),
                                                    {},
                                                    currentTime};
        return DataModels::GlobalPosition::localPoseToGlobalPose(gnssOffset, gnssPose);
    }

    rtl::Quaternion<double> MovementModel::rpyToQuaternion(double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
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

    void MovementModel::quaternionToRPY(rtl::Quaternion<double> q, double& roll, double &pitch, double& yaw) {

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }


    double MovementModel::getSpeedGainFactor(double speed) {
        double y = 1 / ( 1 + std::exp( -0.15 * ( speed-25 ) ) );
        return y;
    }
}