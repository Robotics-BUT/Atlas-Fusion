#include "data_loader/data_models/camera/CameraFrameDataModel.h"
#include <sstream>

namespace AutoDrive {
    namespace DataLoader {

        std::string CameraFrameDataModel::toString() {
            std::stringstream ss;
            ss << "[Camera RGB Data Model] : " << timestamp_;
            return ss.str();
        }
    }
}