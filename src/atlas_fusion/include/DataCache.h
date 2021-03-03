#pragma once

#include "data_models/all.h"
#include "data_loader/DataLoaderIdentifiers.h"

namespace AutoDrive {

    class DataCache {

    public:

        DataCache() = default;

        /* Camera IR */
        void setNewIRFrame(std::shared_ptr<DataModels::CameraIrFrameDataModel> frame);
        std::shared_ptr<DataModels::CameraIrFrameDataModel> getIRFrame(DataLoader::CameraIndentifier id);
        size_t getIRFrameNo(DataLoader::CameraIndentifier id);


    private:

        std::map<DataLoader::CameraIndentifier, const std::shared_ptr<DataModels::CameraIrFrameDataModel>> irFrames_;
        std::map<DataLoader::CameraIndentifier, int> irFramesCounter_;
    };

}