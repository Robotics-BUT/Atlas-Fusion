#include "data_loader/data_models/imu/ImuTempDataModel.h"

#include <sstream>

namespace AutoDrive {
    namespace DataLoader {

        std::string ImuTempDataModel::toString() {
            std::stringstream ss;
            ss << "[Imu Temp Data Model] : " << temperature_;
            return ss.str();
        }
    }
}
