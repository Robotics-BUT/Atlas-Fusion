#include "algorithms/MovementModel.h"
#include "local_map/Frames.h"

namespace AutoDrive::Algorithms {



    void MovementModel::onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data) {
        auto gnssPose = DataModels::GlobalPosition{
                data->getLatitude(),
                data->getLongitude(),
                data->getAltitude(),
                data->getAzimut()};
        auto imuPose = gnssPoseToRootFrame(gnssPose);

        if(!initialized_) {
            initPositionGnss_ = gnssPoseToRootFrame(gnssPose);
            lastImuTimestamp_ = data->getTimestamp();
            lastDquatTimestamp_ = data->getTimestamp();
            initialized_ = true;
        }

        while(positionHistory_.size() >= 100) {
            positionHistory_.pop_front();
        }


        auto offset = DataModels::GlobalPosition::getOffsetBetweenCoords(initPositionGnss_, imuPose);
        auto azimut = data->getAzimut();
        yaw_ = -azimut*M_PI/180;

        cv::Mat measurementX = (cv::Mat_<double>(2, 1) << offset.getPosition().x(), 0);
        cv::Mat measurementY = (cv::Mat_<double>(2, 1) << offset.getPosition().y(), 0);
        cv::Mat measurementZ = (cv::Mat_<double>(2, 1) << offset.getPosition().z(), 0);
        cv::Mat measurementYaw = (cv::Mat_<double>(2, 1) << -azimut*M_PI/180, 0);
        kalmanX_.correct(measurementX);
        kalmanY_.correct(measurementY);
        kalmanZ_.correct(measurementZ);
        kalmanYaw_.correct(measurementYaw);

        //std::cout << kalmanX_.getSpeed() << " " << kalmanY_.getSpeed() << " " << kalmanZ_.getSpeed() << std::endl;

        DataModels::LocalPosition position;
        position.setPositon({kalmanX_.getPosition(), kalmanY_.getPosition(), kalmanZ_.getPosition()});
        auto speed = abs(kalmanY_.getSpeed()) + abs(kalmanX_.getSpeed());
        auto heading = estimateHeading();
        if (speed > 1.0) {
            position.setOrientation(rpyToQuaternion(0, 0, estimateHeading()));
        } else {
            position.setOrientation(rpyToQuaternion(0, 0, -azimut * M_PI / 180));
        }

        //std::cout << "heading: " <<  heading  << " azim: " <<  azimut << std::endl;
        positionHistory_.push_back(position);
    }


    void MovementModel::onImuImuData(std::shared_ptr<DataModels::ImuImuDataModel> data) {

        if(initialized_) {

            auto imuToOriginTf = context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kImuFrame);
            imuToOriginTf.setTranslation({});

            imuProcessor_.setOrientation(data->getOrientation());
            auto linAccNoGrav = imuProcessor_.removeGravitaionAcceleration(data->getLinearAcc());

            double localYaw = 0;
            auto imuOrientation = data->getOrientation();
            quaternionToRPY(imuOrientation, roll_, pitch_, localYaw);

            auto accX = cos(yaw_)*linAccNoGrav.x() - sin(yaw_)*linAccNoGrav.y();
            auto accY = sin(yaw_)*linAccNoGrav.x() + cos(yaw_)*linAccNoGrav.y();

            auto dt = (data->getTimestamp() - lastImuTimestamp_) * 1e-9;
            kalmanX_.predict(dt, accX);
            kalmanY_.predict(dt, accY);
            kalmanZ_.predict(dt, linAccNoGrav.z());

            lastImuTimestamp_ = data->getTimestamp();
        }
    }


    void MovementModel::onImuDquatData(std::shared_ptr<DataModels::ImuDquatDataModel> data) {

        if(initialized_) {

            auto dt = (data->getTimestamp() - lastImuTimestamp_) * 1e-9;
            auto dQuat = data->getDQuat();

            double roll, pitch, yaw;
            quaternionToRPY(dQuat, roll, pitch, yaw);
            kalmanYaw_.predict(dt, yaw);

            lastDquatTimestamp_ = data->getTimestamp();
        }
    }

    DataModels::LocalPosition MovementModel::getPosition() {

        double heading = estimateHeading();
        return DataModels::LocalPosition(
                {kalmanX_.getPosition(), kalmanY_.getPosition(), kalmanZ_.getPosition()},
                rpyToQuaternion(roll_, pitch_, heading/180*M_PI));
    }


    double MovementModel::estimateHeading() {
        double azim = atan2(kalmanY_.getSpeed(), kalmanX_.getSpeed());
        return (azim / M_PI * 180);
    }

    DataModels::GlobalPosition MovementModel::gnssPoseToRootFrame(const DataModels::GlobalPosition gnssPose) {

        auto gnssOffset = context_.tfTree_.transformPointFromFrameToFrame({}, LocalMap::Frames::kGnssAntennaRear, LocalMap::Frames::kImuFrame);
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
}