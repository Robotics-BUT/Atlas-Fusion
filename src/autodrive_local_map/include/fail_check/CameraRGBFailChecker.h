#pragma once

#include "AbstrackFailChecker.h"
#include "data_models/camera/CameraFrameDataModel.h"

namespace AutoDrive::FailCheck {

    /**
     * Validates RGB camera frame data. Currently bypassed.
     */
    class CameraRGBFailChecker : public AbstrackFailChecker{

    public:

        CameraRGBFailChecker() = delete;

        /**
         * Constructor
         * @param context cantainer for global services (timestamps. logging, etc.)
         */
        CameraRGBFailChecker(Context& context)
        : AbstrackFailChecker{context}
        {

        }

        /**
         * Pipe to provide new sensor data into the Camera RGB Fail Checker
         * @param data RGB camera data frame
         */
        void onNewData(std::shared_ptr<DataModels::CameraFrameDataModel> data);

    private:

    };
}

