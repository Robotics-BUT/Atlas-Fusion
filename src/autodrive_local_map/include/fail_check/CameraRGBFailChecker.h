#pragma once

#include "AbstrackFailChecker.h"
#include "data_models/camera/CameraFrameDataModel.h"

namespace AutoDrive::FailCheck {

    class CameraRGBFailChecker : public AbstrackFailChecker{

    public:

        CameraRGBFailChecker() = delete;

        CameraRGBFailChecker(Context& context)
        : AbstrackFailChecker{context}
        {

        }

        void onNewData(std::shared_ptr<DataModels::CameraFrameDataModel>);

    private:

    };
}

