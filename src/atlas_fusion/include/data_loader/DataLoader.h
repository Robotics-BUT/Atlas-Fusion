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

namespace AutoDrive::DataLoader {

        /**
         * Data Loader works as a frontend for the entire data loading sections this class creates the only one bridge
         * between the instances that reads data from data source and the data processing pipeline. Data Loader owns all
         * the sensor specific data loaders and handles the data providing in the correct time order.
         */
        class DataLoader : public AbstractDataLoader {

        public:

            /**
             * Constructor
             * @param context global services container (system time, logging, etc.)
             * @param keepHistoryLength defines the time data is held in memory in nanoseconds
             * before it will be removed
             */
            DataLoader(Context& context, timestamp_type keepHistoryLength)
            : cameraLeftFrontDataLoader_(CameraDataLoader(context, CameraIndentifier::kCameraLeftFront, context.calibFolder_+Files::kCameraLeftFrontCalibYaml))
            , cameraLeftSideDataLoader_(CameraDataLoader(context, CameraIndentifier::kCameraLeftSide, context.calibFolder_+Files::kCameraLeftSideCalibYaml))
            , cameraRightFrontDataLoader_(CameraDataLoader(context, CameraIndentifier::kCameraRightFront, context.calibFolder_+Files::kCameraRightFrontCalibYaml))
            , cameraRightSideDataLoader_(CameraDataLoader(context, CameraIndentifier::kCameraRightSide, context.calibFolder_+Files::kCameraRightSideCalibYaml))
            , cameraIrDataLoader_(CameraDataLoader(context, CameraIndentifier::kCameraIr, context.calibFolder_+Files::kCameraIrCalibYaml))
            , leftLidarDataLoader_(LidarDataLoader(context, LidarIdentifier::kLeftLidar))
            , rightLidarDataLoader_(LidarDataLoader(context, LidarIdentifier::kRightLidar))
            , centerLidarDataLoader_(LidarDataLoader(context, LidarIdentifier::kCenterLidar))
            , gnssPoseDataLoader_(GnssDataLoader(context, GnssLoaderIdentifier::kPose))
            , gnssTimeDataLoader_(GnssDataLoader(context, GnssLoaderIdentifier::kTime))
            , imuDquatDataLoader_(ImuDataLoader(context, ImuLoaderIdentifier::kDQuat))
            , imuGnssDataLoader_(ImuDataLoader(context, ImuLoaderIdentifier::kGnss))
            , imuImuDataLoader_(ImuDataLoader(context, ImuLoaderIdentifier::kImu))
            , imuMagDataLoader_(ImuDataLoader(context, ImuLoaderIdentifier::kMag))
            , imuPressDataLoader_(ImuDataLoader(context, ImuLoaderIdentifier::kPressure))
            , imuTempDataLoader_(ImuDataLoader(context, ImuLoaderIdentifier::kTemp))
            , imuTimeDataLoader_(ImuDataLoader(context, ImuLoaderIdentifier::kTime))
            , radarTiDataLoader_(RadarTiDataLoader(context, RadarIdentifier::kRadarTi))
            , context_{context}
            , keepHistoryLength_(keepHistoryLength) {

                dataLoaders_ = {
                        &cameraLeftFrontDataLoader_,
                        &cameraLeftSideDataLoader_,
                        &cameraRightFrontDataLoader_,
                        &cameraRightSideDataLoader_,
                        &cameraIrDataLoader_,
                        &leftLidarDataLoader_,
                        &rightLidarDataLoader_,
                        &centerLidarDataLoader_,
                        &gnssPoseDataLoader_,
                        &gnssTimeDataLoader_,
                        &imuDquatDataLoader_,
                        &imuGnssDataLoader_,
                        &imuImuDataLoader_,
                        &imuMagDataLoader_,
                        &imuPressDataLoader_,
                        &imuTempDataLoader_,
                        &imuTimeDataLoader_,
                        &radarTiDataLoader_,
                };

                cameraDataLoaders_ = {
                        &cameraLeftFrontDataLoader_,
                        &cameraLeftSideDataLoader_,
                        &cameraRightFrontDataLoader_,
                        &cameraRightSideDataLoader_,
                        &cameraIrDataLoader_,
                };
            }


            bool initData(const std::string& path) override;
            timestamp_type getLowestTimestamp() override;

            void startAsyncDataLoading(size_t maxCacheSize);
            std::shared_ptr<DataModels::GenericDataModel> getNextFrameAsync();
            std::string toString() override;
            uint64_t getDataSize() override;
            bool isOnEnd() const override;
            void setPose(timestamp_type) override;
            void releaseOldData(timestamp_type keepHistory) override;
            void clear() override;

            /**
             * Method converts camera frame name into the Camera Data Loader specific identifier.
             * @return Identifier that identifies Camera Data Loader related to the given camera frame.
             */
            CameraIndentifier getCameraIDfromFrame(const FrameType&);

            /**
             * Method allows to request the camera's calibration parameters for specific camera sensor
             * @param id camera sensor identifier
             * @return calibration data for given camera sensor
             */
            DataModels::CameraCalibrationParamsDataModel getCameraCalibDataForCameraID(CameraIndentifier id);

        private:

            CameraDataLoader cameraLeftFrontDataLoader_;
            CameraDataLoader cameraLeftSideDataLoader_;
            CameraDataLoader cameraRightFrontDataLoader_;
            CameraDataLoader cameraRightSideDataLoader_;

            CameraDataLoader cameraIrDataLoader_;

            LidarDataLoader leftLidarDataLoader_;
            LidarDataLoader rightLidarDataLoader_;
            LidarDataLoader centerLidarDataLoader_;

            GnssDataLoader gnssPoseDataLoader_;
            GnssDataLoader gnssTimeDataLoader_;

            ImuDataLoader imuDquatDataLoader_;
            ImuDataLoader imuGnssDataLoader_;
            ImuDataLoader imuImuDataLoader_;
            ImuDataLoader imuMagDataLoader_;
            ImuDataLoader imuPressDataLoader_;
            ImuDataLoader imuTempDataLoader_;
            ImuDataLoader imuTimeDataLoader_;

            RadarTiDataLoader radarTiDataLoader_;

            std::vector<AbstractDataLoader*> dataLoaders_;
            std::vector<CameraDataLoader*> cameraDataLoaders_;

            // Queue of cached frames, second parameter indicates whether the frame has been read;
            std::deque<std::pair<std::shared_ptr<DataModels::GenericDataModel>, bool>> dataQueue_{};

            Context& context_;
            timestamp_type keepHistoryLength_;

            bool checkRecordConsistency(const std::string& path);
            bool loadRecord(const std::string& path);

            std::shared_ptr<DataModels::GenericDataModel> getNextData() override;
        };
    }