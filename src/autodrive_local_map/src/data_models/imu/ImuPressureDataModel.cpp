#include "data_models/imu/ImuPressureDataModel.h"

#include <sstream>

namespace AutoDrive::DataModels {

    std::string ImuPressureDataModel::toString() {
        std::stringstream ss;
        ss << "[Imu Pressure Data Model] : " << pressure_;
        return ss.str();
    }
}