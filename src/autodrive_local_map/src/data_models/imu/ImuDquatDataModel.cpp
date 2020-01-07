#include "data_models/imu/ImuDquatDataModel.h"

#include <sstream>

namespace AutoDrive::DataModels {

    std::string ImuDquatDataModel::toString() {
        std::stringstream ss;
        ss << "[Imu Dquat Data Model] : "
           << dRotation_.x() << " " << dRotation_.y() << " " << dRotation_.z() << " " << dRotation_.w();
        return ss.str();
    }
}