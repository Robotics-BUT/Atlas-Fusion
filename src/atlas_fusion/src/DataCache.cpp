#include "DataCache.h"
#include "RustStyle.h"

namespace AutoDrive{

    void DataCache::setNewIRFrame(const std::shared_ptr<DataModels::CameraIrFrameDataModel> frame) {
        let id = frame->getCameraIdentifier();
        irFrames_.insert({id, frame});
        if (irFramesCounter_.count(id) == 0) {
            irFramesCounter_[id] = 0;
        } else {
            irFramesCounter_.at(id) += 1;
        }
    }

    std::shared_ptr<DataModels::CameraIrFrameDataModel> DataCache::getIRFrame(DataLoader::CameraIndentifier id) {
        if (irFrames_.find(id) != irFrames_.end()) {
            return irFrames_.at(id);
        }
        return nullptr;
    }

    size_t DataCache::getIRFrameNo(DataLoader::CameraIndentifier id) {
        if (irFramesCounter_.count(id) == 0) {
            return -1;
        }
        return irFramesCounter_.at(id);
    }

}