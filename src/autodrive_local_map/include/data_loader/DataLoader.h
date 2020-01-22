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
#include "Context.h"

namespace AutoDrive {
    namespace DataLoader {

        class DataLoader : public AbstractDataLoader {

        public:

            DataLoader(Context& context, timestamp_type keepHistoryLength)
            : cameraLeftFrontDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraLeftFront, context.calibFolder_+Files::kCameraLeftFrontCalibYaml))
            , cameraLeftSideDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraLeftSide, context.calibFolder_+Files::kCameraLeftSideCalibYaml))
            , cameraRightFrontDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraRightFront, context.calibFolder_+Files::kCameraRightFrontCalibYaml))
            , cameraRightSideDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraRightSide, context.calibFolder_+Files::kCameraRightSideCalibYaml))
            , cameraIrDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraIr, context.calibFolder_+Files::kCameraIrCalibYaml))
            , leftLidarDataLoader_(std::make_shared<LidarDataLoader>(context, LidarIdentifier::kLeftLidar))
            , rightLidarDataLoader_(std::make_shared<LidarDataLoader>(context, LidarIdentifier::kRightLidar))
            , gnssPoseDataLoader_(std::make_shared<GnssDataLoader>(context, GnssLoaderIdentifier::kPose))
            , gnssTimeDataLoader_(std::make_shared<GnssDataLoader>(context, GnssLoaderIdentifier::kTime))
            , imuDquatDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kDQuat))
            , imuGnssDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kGnss))
            , imuImuDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kImu))
            , imuMagDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kMag))
            , imuPressDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kPressure))
            , imuTempDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kTemp))
            , imuTimeDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kTime))
            , context_{context}
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

                cameraDataLoaders_ = {
                        cameraLeftFrontDataLoader_,
                        cameraLeftSideDataLoader_,
                        cameraRightFrontDataLoader_,
                        cameraRightSideDataLoader_,
                        cameraIrDataLoader_,
                };
            }

            bool loadData(std::string path) override;
            timestamp_type getLowestTimestamp() override;
            std::shared_ptr<DataModels::GenericDataModel> getNextData() override;
            std::string toString() override;
            uint64_t getDataSize() override;
            bool isOnEnd() override;
            void setPose(timestamp_type) override;
            void releaseOldData(timestamp_type keepHistory) override;
            void clear() override;

            CameraIndentifier getCameraIDfromFrame(const std::string&);
            std::shared_ptr<DataModels::GenericDataModel> getCameraCalibDataForCameraID(CameraIndentifier id);

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
            std::vector<std::shared_ptr<CameraDataLoader>> cameraDataLoaders_;


            Context& context_;
            timestamp_type keepHistoryLength_;

            bool checkRecordConsistency(std::string& path);
            bool loadRecord(std::string& path);

        };
    }
}