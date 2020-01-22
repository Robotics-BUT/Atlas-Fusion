#pragma once

#include "AbstrackFailChecker.h"
#include "data_models/camera/CameraIrFrameDataModel.h"

namespace AutoDrive::FailCheck {

    class CameraIrFailChecker : public AbstrackFailChecker{

    public:

        CameraIrFailChecker() = delete;

        CameraIrFailChecker(Context& context)
                : AbstrackFailChecker{context}
        {

        }

        void onNewData(std::shared_ptr<DataModels::CameraIrFrameDataModel>);

    private:

    };
}

