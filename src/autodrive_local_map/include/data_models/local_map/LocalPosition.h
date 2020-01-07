#pragma once

#include <rtl/Vector3D.h>
#include <rtl/Quaternion.h>

namespace AutoDrive::DataModels {

    class LocalPosition {

    public:

        LocalPosition(rtl::Vector3D<double> positon = {}, rtl::Quaternion<double> orientation={})
        : position_(std::move(positon))
        , orientation_(std::move(orientation)) {

        }

        rtl::Vector3D<double> getPosition() const { return position_; };
        rtl::Quaternion<double> getOrientation() const { return orientation_;};

        void setX(double x) {position_.setX(x);};
        void setY(double y) {position_.setY(y);};
        void setZ(double z) {position_.setZ(z);};

        void setPositon(rtl::Vector3D<double> pose) { position_ = pose;};
        void setOrientation(rtl::Quaternion<double> orientation) { orientation_ = orientation; };
    private:

        rtl::Vector3D<double> position_;
        rtl::Quaternion<double> orientation_;

    };

}

