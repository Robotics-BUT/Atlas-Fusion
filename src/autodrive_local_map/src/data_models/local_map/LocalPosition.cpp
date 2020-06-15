#include "data_models/local_map/LocalPosition.h"

namespace AutoDrive::DataModels {


    rtl::RigidTf3D<double> LocalPosition::toTf() const {
        return {getOrientation(), getPosition()};
    }

    LocalPosition LocalPosition::operator+(LocalPosition& other) {

        return LocalPosition{{position_.x() + other.getPosition().x(),
                              position_.y() + other.getPosition().y(),
                              position_.z() + other.getPosition().z()},
                             {orientation_ * other.getOrientation()},
                             timestamp_ + other.getTimestamp()};
    }

    LocalPosition LocalPosition::operator-(LocalPosition& other) {
        return LocalPosition{{position_.x() - other.getPosition().x(),
                              position_.y() - other.getPosition().y(),
                              position_.z() - other.getPosition().z()},
                             {other.getOrientation().inverted() * orientation_},
                             timestamp_ - other.getTimestamp()};
    }

    std::string LocalPosition::toString() {

        std::stringstream ss;
        ss << position_.x() << " " << position_.y() << " " << position_.z() << " | "
           << orientation_.x() << " " << orientation_.y() << " " << orientation_.z() << " " << orientation_.w() << " | "
           << timestamp_;
        return ss.str();
    }

}