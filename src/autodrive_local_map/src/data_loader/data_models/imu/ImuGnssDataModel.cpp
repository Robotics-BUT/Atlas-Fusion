#include "data_loader/data_models/imu/ImuGnssDataModel.h"

#include <sstream>

namespace AutoDrive {
    namespace DataLoader {

        std::string ImuGnssDataModel::toString() {
            std::stringstream ss;
            ss << "[Imu Gnss Data Model] : "
               << latitude_ << " " << longitude_ << " " << altitude_;
            return ss.str();
        }
    }
}