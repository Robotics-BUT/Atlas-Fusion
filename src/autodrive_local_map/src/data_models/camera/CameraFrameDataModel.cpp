#include "data_models/camera/CameraFrameDataModel.h"
#include <sstream>

namespace AutoDrive::DataModels {

    std::string CameraFrameDataModel::toString() {
        std::stringstream ss;
        ss << "[Camera RGB Data Model] : " << timestamp_;
        return ss.str();
    }
}