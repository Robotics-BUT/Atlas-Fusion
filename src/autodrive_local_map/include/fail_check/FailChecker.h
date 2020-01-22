#pragma once

#include "Context.h"
#include "AbstrackFailChecker.h"
#include "CameraRGBFailChecker.h"
#include "CameraIrFailChecker.h"
#include "GnssFailChecker.h"
#include "LidarFailChecker.h"
#include "ImuFailChecker.h"

namespace AutoDrive::FailCheck {

    class FailChecker {

    public:

        enum class SensorFailCheckID{
            kCameraLeftFront,
            kCameraLeftSide,
            kCameraRightFront,
            kCameraRightSide,
            kCameraIr,
            kLidarLeft,
            kLidarRight,
            kImu,
            kGnss,
            kRadar,
            kErr,
        };

        FailChecker() = delete;
        explicit FailChecker(Context& context)
        : context_{context} {

            failCheckers_[SensorFailCheckID::kCameraLeftFront] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraRGBFailChecker>(context));
            failCheckers_[SensorFailCheckID::kCameraLeftSide] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraRGBFailChecker>(context));
            failCheckers_[SensorFailCheckID::kCameraRightFront] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraRGBFailChecker>(context));
            failCheckers_[SensorFailCheckID::kCameraRightSide] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraRGBFailChecker>(context));
            failCheckers_[SensorFailCheckID::kCameraIr] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraIrFailChecker>(context));

            failCheckers_[SensorFailCheckID::kLidarLeft] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<LidarFailChecker>(context));
            failCheckers_[SensorFailCheckID::kLidarRight] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<LidarFailChecker>(context));

            failCheckers_[SensorFailCheckID::kImu] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<ImuFailChecker>(context));
            failCheckers_[SensorFailCheckID::kGnss] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<GnssFailChecker>(context));
//            failCheckers_[SensorFailCheckID::kRadar] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<RadarFailChecker>(context));
        }

        void onNewData(std::shared_ptr<DataModels::GenericDataModel>, SensorFailCheckID sensor);
        float getSensorStatus(SensorFailCheckID);

        SensorFailCheckID frameToFailcheckID(std::string frame);

    protected:

        Context& context_;
        std::map<SensorFailCheckID, std::shared_ptr<FailCheck::AbstrackFailChecker>> failCheckers_;
    };

}
