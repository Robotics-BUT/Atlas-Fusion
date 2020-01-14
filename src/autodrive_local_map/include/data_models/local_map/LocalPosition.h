#pragma once

#include <rtl/Vector3D.h>
#include <rtl/Quaternion.h>

namespace AutoDrive::DataModels {

    class LocalPosition {

    public:

        explicit LocalPosition(rtl::Vector3D<double> positon, rtl::Quaternion<double> orientation, uint64_t timestamp)
        : position_(std::move(positon))
        , orientation_(std::move(orientation))
        , timestamp_{timestamp} {

        }

        rtl::Vector3D<double> getPosition() const { return position_; };
        rtl::Quaternion<double> getOrientation() const { return orientation_;};

        void setX(double x) {position_.setX(x);};
        void setY(double y) {position_.setY(y);};
        void setZ(double z) {position_.setZ(z);};

        void setPositon(rtl::Vector3D<double> pose) { position_ = pose;};
        void setOrientation(rtl::Quaternion<double> orientation) { orientation_ = orientation; };

        uint64_t getTimestamp() const {return timestamp_;};

        LocalPosition operator+(LocalPosition& other);
        LocalPosition operator-(LocalPosition& other);

        std::string toString();


    private:

        rtl::Vector3D<double> position_;
        rtl::Quaternion<double> orientation_;
        uint64_t timestamp_;

    };

}

