#pragma once

#include "data_models/local_map/LocalPosition.h"

namespace AutoDrive::DataModels {

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

}