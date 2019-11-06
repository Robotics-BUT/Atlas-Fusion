#include "data_loader/data_models/imu/ImuPressureDataModel.h"

#include <sstream>

namespace AutoDrive {
    namespace DataLoader {

        std::string ImuPressureDataModel::toString() {
            std::stringstream ss;
            ss << "[Imu Pressure Data Model] : " << pressure_;
            return ss.str();
        }
    }
}