#include "data_models/gnss/GnssPoseDataModel.h"

#include <sstream>

namespace AutoDrive::DataModels {

    std::string GnssPoseDataModel::toString() {
        std::stringstream ss;
        ss << "[Gnss Pose Data Model] : " << timestamp_
           << latitude_ << " " << longitude_ << " " << altitude_ << " " << azimut_;
        return ss.str();
    }
}