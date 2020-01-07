#include "data_models/camera/CameraIrFrameDataModel.h"



namespace AutoDrive::DataModels {

    std::string CameraIrFrameDataModel::toString() {
        std::stringstream ss;
        ss << "[Camera IR Data Model] : " << timestamp_;
        return ss.str();
    }
}