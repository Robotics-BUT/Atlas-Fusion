#include "data_loader/data_models/imu/ImuMagDataModel.h"

#include <sstream>

namespace AutoDrive {
    namespace DataLoader {

        std::string ImuMagDataModel::toString() {
            std::stringstream ss;
            ss << "[Imu Mag Data Model] : "
               << mag_.x() << " " << mag_.y() << " " << mag_.z();
            return ss.str();
        }
    }
}