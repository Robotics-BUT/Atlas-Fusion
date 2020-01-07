#include "data_models/imu/ImuImuDataModel.h"

#include <sstream>

namespace AutoDrive::DataModels {

    std::string ImuImuDataModel::toString() {
        std::stringstream ss;
        ss << "[Imu Imu Data Model] : "
           << linearAcc_.x() << " " << linearAcc_.y() << " " << linearAcc_.z()
           << angularVel_.x() << " " << angularVel_.y() << " " << angularVel_.z()
           << orientation_.x() << " " << orientation_.y() << " " << orientation_.z() << " " << orientation_.w();
        return ss.str();
    }
}