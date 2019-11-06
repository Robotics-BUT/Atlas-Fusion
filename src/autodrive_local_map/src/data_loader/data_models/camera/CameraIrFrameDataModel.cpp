#include "data_loader/data_models/camera/CameraIrFrameDataModel.h"



namespace AutoDrive {
    namespace DataLoader {


        std::string CameraIrFrameDataModel::toString() {
            std::stringstream ss;
            ss << "[Camera IR Data Model] : " << timestamp_;
            return ss.str();
        }
    }
}