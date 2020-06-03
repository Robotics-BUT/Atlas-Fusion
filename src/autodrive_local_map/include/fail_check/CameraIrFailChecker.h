#pragma once

#include "AbstrackFailChecker.h"
#include "data_models/camera/CameraIrFrameDataModel.h"

namespace AutoDrive::FailCheck {

    /**
     * Validates IR camera frame data. Currently bypassed.
     */
    class CameraIrFailChecker : public AbstrackFailChecker{

    public:

        CameraIrFailChecker() = delete;

        /**
         * Constructor
         * @param context cantainer for global services (timestamps. logging, etc.)
         */
        CameraIrFailChecker(Context& context)
                : AbstrackFailChecker{context}
        {

        }

        /**
         * Pipe to provide new sensor data into the Camera RGB Fail Checker
         * @param data IR camera data frame
         */
        void onNewData(std::shared_ptr<DataModels::CameraIrFrameDataModel> data);

    private:

    };
}

