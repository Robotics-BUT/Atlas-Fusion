/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <vector>
#include <memory>
#include "data_models/all.h"
#include "data_models/GenericDataModel.h"
#include "CameraDataLoader.h"
#include "LidarDataLoader.h"
#include "GnssDataLoader.h"
#include "ImuDataLoader.h"
#include "RadarTiDataLoader.h"
#include "AbstractDataLoader.h"
#include "Context.h"

namespace AtlasFusion {
    namespace DataLoader {

        /**
         * Data Loader works as a frontend for the entire data loading sections this class creates the only one bridge
         * between the intancess that reads data from data source and the data processin pipeline. Data Loader owns all
         * the sensor specific data loaders and handles the data providing in the correct time order.
         */
        class DataLoader : public AbstractDataLoader {

        public:

            /**
             * Constructor
             * @param context global services container (system time, logging, etc.)
             * @param keepHistoryLength defines in nanoseconds how long should be the past data be holded in momory
             * before it will be removed
             */
            DataLoader(Context& context, timestamp_type keepHistoryLength)
            : cameraLeftFrontDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraLeftFront, context.calibFolder_+Files::kCameraLeftFrontCalibYaml))
            , cameraLeftSideDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraLeftSide, context.calibFolder_+Files::kCameraLeftSideCalibYaml))
            , cameraRightFrontDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraRightFront, context.calibFolder_+Files::kCameraRightFrontCalibYaml))
            , cameraRightSideDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraRightSide, context.calibFolder_+Files::kCameraRightSideCalibYaml))
            , cameraIrDataLoader_(std::make_shared<CameraDataLoader>(context, CameraIndentifier::kCameraIr, context.calibFolder_+Files::kCameraIrCalibYaml))
            , leftLidarDataLoader_(std::make_shared<LidarDataLoader>(context, LidarIdentifier::kLeftLidar))
            , rightLidarDataLoader_(std::make_shared<LidarDataLoader>(context, LidarIdentifier::kRightLidar))
            , centerLidarDataLoader_(std::make_shared<LidarDataLoader>(context, LidarIdentifier::kCenterLidar))
            , gnssPoseDataLoader_(std::make_shared<GnssDataLoader>(context, GnssLoaderIdentifier::kPose))
            , gnssTimeDataLoader_(std::make_shared<GnssDataLoader>(context, GnssLoaderIdentifier::kTime))
            , imuDquatDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kDQuat))
            , imuGnssDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kGnss))
            , imuImuDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kImu))
            , imuMagDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kMag))
            , imuPressDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kPressure))
            , imuTempDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kTemp))
            , imuTimeDataLoader_(std::make_shared<ImuDataLoader>(context, ImuLoaderIdentifier::kTime))
            , radarTiDataLoader_(std::make_shared<RadarTiDataLoader>(context, RadarIdentifier::kRadarTi))
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
                        centerLidarDataLoader_,
                        gnssPoseDataLoader_,
                        gnssTimeDataLoader_,
                        imuDquatDataLoader_,
                        imuGnssDataLoader_,
                        imuImuDataLoader_,
                        imuMagDataLoader_,
                        imuPressDataLoader_,
                        imuTempDataLoader_,
                        imuTimeDataLoader_,
                        radarTiDataLoader_,
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

            /**
             * Method converts camera frame name into the Camera Data Loader specific identifier.
             * @return Identifier that identifies Camera Data Loader related to the given camera frame.
             */
            CameraIndentifier getCameraIDfromFrame(const std::string&);

            /**
             * Method allows to request the camera's calibration parameters for specific camera sensor
             * @param id camera sensor identifier
             * @return calibration data for given camera sensor
             */
            std::shared_ptr<DataModels::GenericDataModel> getCameraCalibDataForCameraID(CameraIndentifier id);

        private:

            std::shared_ptr<CameraDataLoader> cameraLeftFrontDataLoader_;
            std::shared_ptr<CameraDataLoader> cameraLeftSideDataLoader_;
            std::shared_ptr<CameraDataLoader> cameraRightFrontDataLoader_;
            std::shared_ptr<CameraDataLoader> cameraRightSideDataLoader_;

            std::shared_ptr<CameraDataLoader> cameraIrDataLoader_;

            std::shared_ptr<LidarDataLoader> leftLidarDataLoader_;
            std::shared_ptr<LidarDataLoader> rightLidarDataLoader_;
            std::shared_ptr<LidarDataLoader> centerLidarDataLoader_;

            std::shared_ptr<GnssDataLoader> gnssPoseDataLoader_;
            std::shared_ptr<GnssDataLoader> gnssTimeDataLoader_;

            std::shared_ptr<ImuDataLoader> imuDquatDataLoader_;
            std::shared_ptr<ImuDataLoader> imuGnssDataLoader_;
            std::shared_ptr<ImuDataLoader> imuImuDataLoader_;
            std::shared_ptr<ImuDataLoader> imuMagDataLoader_;
            std::shared_ptr<ImuDataLoader> imuPressDataLoader_;
            std::shared_ptr<ImuDataLoader> imuTempDataLoader_;
            std::shared_ptr<ImuDataLoader> imuTimeDataLoader_;

            std::shared_ptr<RadarTiDataLoader> radarTiDataLoader_;

            std::vector<std::shared_ptr<AbstractDataLoader>> dataLoaders_;
            std::vector<std::shared_ptr<CameraDataLoader>> cameraDataLoaders_;


            Context& context_;
            timestamp_type keepHistoryLength_;

            bool checkRecordConsistency(std::string& path);
            bool loadRecord(std::string& path);
        };
    }
}