#include "data_models/imu/ImuGnssDataModel.h"

#include <sstream>

namespace AutoDrive::DataModels {

    std::string ImuGnssDataModel::toString() {
        std::stringstream ss;
        ss << "[Imu Gnss Data Model] : "
           << latitude_ << " " << longitude_ << " " << altitude_;
        return ss.str();
    }
}