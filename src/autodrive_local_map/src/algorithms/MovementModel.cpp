#include "algorithms/MovementModel.h"
#include "local_map/Frames.h"
#include <rtl/Transformation3D.h>

namespace AutoDrive::Algorithms {


    /// Hooks

    void MovementModel::onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data) {

        currentTime = data->getTimestamp();

        auto gnssPose = DataModels::GlobalPosition{ data->getLatitude(), data->getLongitude(), data->getAltitude(), 0};
        auto imuPose = gnssPoseToRootFrame(gnssPose);
        auto heading = azimuthToHeading(data->getAzimut());

        if(!poseInitialized_) {
            initPositionGnss_ = gnssPoseToRootFrame(gnssPose);
            lastImuTimestamp_ = data->getTimestamp();
            lastDquatTimestamp_ = data->getTimestamp();
            poseInitialized_ = true;
        }

        if(heading != 0) {

            if(!headingInitialized_) {
                orientation_ = rpyToQuaternion(0,0,heading);
                headingInitialized_ = true;
            }

            orientation_ = orientation_.slerp(rpyToQuaternion(0,0,heading), 0.01);
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
                                           orientation_,
                                           data->getTimestamp()};

        positionHistory_.push_back(position);
    }


    void MovementModel::onImuImuData(std::shared_ptr<DataModels::ImuImuDataModel> data) {

        currentTime = data->getTimestamp();
        if(poseInitialized_) {

            imuProcessor_.setOrientation(data->getOrientation());
            auto linAccNoGrav = imuProcessor_.removeGravitaionAcceleration(data->getLinearAcc());

            auto orientatnionTF = rtl::Transformation3D<double>{orientation_,{}};
            auto rotatedLinAccNoGrav = orientatnionTF(linAccNoGrav);

            auto dt = (data->getTimestamp() - lastImuTimestamp_) * 1e-9;
            kalmanX_.predict(dt, rotatedLinAccNoGrav.x());
            kalmanY_.predict(dt, rotatedLinAccNoGrav.y());
            kalmanZ_.predict(dt, rotatedLinAccNoGrav.z());
            lastImuTimestamp_ = data->getTimestamp();
        }
    }


    void MovementModel::onImuDquatData(std::shared_ptr<DataModels::ImuDquatDataModel> data) {

        currentTime = data->getTimestamp();

        if(poseInitialized_) {
            orientation_ = orientation_ * data->getDQuat();
            lastDquatTimestamp_ = data->getTimestamp();
        }
    }



    /// Getters

    DataModels::LocalPosition MovementModel::getPosition() const {

        if(positionHistory_.empty()){
            return DataModels::LocalPosition{{},{},0};
        }
        return positionHistory_.back();
    }


    double MovementModel::getSpeed() const {
        return std::sqrt( std::pow(kalmanX_.getSpeed(),2) + std::pow(kalmanY_.getSpeed(),2) + std::pow(kalmanZ_.getSpeed(),2) );
    }


    rtl::Vector3D<double> MovementModel::getVectorSpeed() const {
        return {kalmanX_.getSpeed(), kalmanY_.getSpeed(), kalmanZ_.getSpeed()};
    }


    double MovementModel::getHeading() const {
        return estimateHeading();
    }


    rtl::Quaternion<double> MovementModel::getOrientation() const {
        return orientation_;
    }


    /// Others

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


    double MovementModel::estimateHeading() const {

        double roll, pitch, yaw;
        quaternionToRPY(orientation_, roll, pitch, yaw);
        return yaw;
    }


    double MovementModel::estimateSpeedHeading() const {
        return atan2(kalmanY_.getSpeed(), kalmanX_.getSpeed());
    }

    [[deprecated]] float MovementModel::getSpeedAzimOffset() {
        auto speedHeading = atan2(kalmanY_.getSpeed(), kalmanX_.getSpeed());
        return speedHeading;
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

    void MovementModel::quaternionToRPY(rtl::Quaternion<double> q, double& roll, double &pitch, double& yaw) const {

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


    double MovementModel::getSpeedGainFactor(double speed) const {
        double y = 1 / ( 1 + std::exp( -0.15 * ( speed-25 ) ) );
        return y;
    }


    double MovementModel::azimuthToHeading(double azimuth) const {
        return -azimuth * M_PI / 180;
    }
}