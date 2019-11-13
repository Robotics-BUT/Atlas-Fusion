#pragma once

#include <vector>
#include <memory>
#include "data_models/all.h"
#include "data_models/GenericDataModel.h"
#include "CameraDataLoader.h"
#include "LidarDataLoader.h"
#include "GnssDataLoader.h"
#include "ImuDataLoader.h"
#include "AbstractDataLoader.h"
#include "LogService.h"

namespace AutoDrive {
    namespace DataLoader {

        class DataLoader : public AbstractDataLoader {

        public:

            DataLoader(LogService& logger, timestamp_type keepHistoryLength)
            : cameraLeftFrontDataLoader_(std::make_shared<CameraDataLoader>(CameraIndentifier::kCameraLeftFront))
            , cameraLeftSideDataLoader_(std::make_shared<CameraDataLoader>(CameraIndentifier::kCameraLeftSide))
            , cameraRightFrontDataLoader_(std::make_shared<CameraDataLoader>(CameraIndentifier::kCameraRightFront))
            , cameraRightSideDataLoader_(std::make_shared<CameraDataLoader>(CameraIndentifier::kCameraRightSide))
            , cameraIrDataLoader_(std::make_shared<CameraDataLoader>(CameraIndentifier::kCameraIr))
            , leftLidarDataLoader_(std::make_shared<LidarDataLoader>(LidarIdentifier::kLeftLidar))
            , rightLidarDataLoader_(std::make_shared<LidarDataLoader>(LidarIdentifier::kRightLidar))
            , gnssPoseDataLoader_(std::make_shared<GnssDataLoader>(GnssLoaderIdentifier::kPose))
            , gnssTimeDataLoader_(std::make_shared<GnssDataLoader>(GnssLoaderIdentifier::kTime))
            , imuDquatDataLoader_(std::make_shared<ImuDataLoader>(ImuLoaderIdentifier::kDQuat))
            , imuGnssDataLoader_(std::make_shared<ImuDataLoader>(ImuLoaderIdentifier::kGnss))
            , imuImuDataLoader_(std::make_shared<ImuDataLoader>(ImuLoaderIdentifier::kImu))
            , imuMagDataLoader_(std::make_shared<ImuDataLoader>(ImuLoaderIdentifier::kMag))
            , imuPressDataLoader_(std::make_shared<ImuDataLoader>(ImuLoaderIdentifier::kPressure))
            , imuTempDataLoader_(std::make_shared<ImuDataLoader>(ImuLoaderIdentifier::kTemp))
            , imuTimeDataLoader_(std::make_shared<ImuDataLoader>(ImuLoaderIdentifier::kTime))
            , logger_(logger)
            , keepHistoryLength_(keepHistoryLength) {

                dataLoaders_ = {
                        cameraLeftFrontDataLoader_,
                        cameraLeftSideDataLoader_,
                        cameraRightFrontDataLoader_,
                        cameraRightSideDataLoader_,
                        cameraIrDataLoader_,
                        leftLidarDataLoader_,
                        rightLidarDataLoader_,
                        gnssPoseDataLoader_,
                        gnssTimeDataLoader_,
                        imuDquatDataLoader_,
                        imuGnssDataLoader_,
                        imuImuDataLoader_,
                        imuMagDataLoader_,
                        imuPressDataLoader_,
                        imuTempDataLoader_,
                        imuTimeDataLoader_,
                };
            }

            bool loadData(std::string path) override;
            timestamp_type getLowestTimestamp() override;
            std::shared_ptr<GenericDataModel> getNextData() override;
            std::string toString() override;
            uint64_t getDataSize() override;
            bool isOnEnd() override;
            void setPose(timestamp_type) override;
            void releaseOldData(timestamp_type keepHistory) override;
            void clear() override;


        private:

            std::shared_ptr<CameraDataLoader> cameraLeftFrontDataLoader_;
            std::shared_ptr<CameraDataLoader> cameraLeftSideDataLoader_;
            std::shared_ptr<CameraDataLoader> cameraRightFrontDataLoader_;
            std::shared_ptr<CameraDataLoader> cameraRightSideDataLoader_;

            std::shared_ptr<CameraDataLoader> cameraIrDataLoader_;

            std::shared_ptr<LidarDataLoader> leftLidarDataLoader_;
            std::shared_ptr<LidarDataLoader> rightLidarDataLoader_;

            std::shared_ptr<GnssDataLoader> gnssPoseDataLoader_;
            std::shared_ptr<GnssDataLoader> gnssTimeDataLoader_;

            std::shared_ptr<ImuDataLoader> imuDquatDataLoader_;
            std::shared_ptr<ImuDataLoader> imuGnssDataLoader_;
            std::shared_ptr<ImuDataLoader> imuImuDataLoader_;
            std::shared_ptr<ImuDataLoader> imuMagDataLoader_;
            std::shared_ptr<ImuDataLoader> imuPressDataLoader_;
            std::shared_ptr<ImuDataLoader> imuTempDataLoader_;
            std::shared_ptr<ImuDataLoader> imuTimeDataLoader_;

            std::vector<std::shared_ptr<AbstractDataLoader>> dataLoaders_;

            LogService& logger_;
            timestamp_type keepHistoryLength_;

            bool checkRecordConsistency(std::string& path);
            bool loadRecord(std::string& path);

        };
    }
}