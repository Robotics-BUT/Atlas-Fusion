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

#include <chrono>

#include "MapBuilder.h"

#include "data_models/all.h"
#include "data_models/DataModelTypes.h"

#include "algorithms/Projector.h"

namespace AutoDrive {


    void MapBuilder::loadData(const std::string& dataFolder) {
        visualizationHandler_.drawTestingCube();

        dataLoader_.loadData(dataFolder);
        std::cout << "Total No. of loaded data: " << dataLoader_.getDataSize() << std::endl;

        std::vector<std::string> cameraFrames = {
                LocalMap::Frames::kCameraLeftFront,
                LocalMap::Frames::kCameraLeftSide,
                LocalMap::Frames::kCameraRightFront,
                LocalMap::Frames::kCameraRightSide,
                LocalMap::Frames::kCameraIr,
        };

        for(const auto& frame : cameraFrames) {
            auto cameraID = dataLoader_.getCameraIDfromFrame(frame);
            auto params = dataLoader_.getCameraCalibDataForCameraID(cameraID);
            auto tf = context_.tfTree_.getTransformationForFrame(frame);

            if(params->getType() == DataModels::DataModelTypes::kCameraCalibrationParamsDataModelType) {
                auto paramModel = std::dynamic_pointer_cast<DataModels::CameraCalibrationParamsDataModel>(params);
                visualizationHandler_.setCameraCalibParamsForCameraId(paramModel, frame);

                auto intrinsicParams = paramModel->getMatIntrinsicParams();
                auto distortionParams =  paramModel->getMatDistortionParams();

                auto projector = std::make_shared<Algorithms::Projector>(intrinsicParams, distortionParams, tf);
                if(frame == LocalMap::Frames::kCameraIr) {
                    yoloIrReprojector_.setupCameraProjector(projector);
                }

                depthMap_.addProjector(projector, cameraID);
                detectionProcessor_.addProjector(projector, frame);
            } else {
                context_.logger_.warning("Unable to read camera calib data");
            }
        }
    }

    void MapBuilder::buildMap() {

        int64_t last_system_ts = 0;
        uint64_t last_data_ts = 0;

        DataModels::LocalPosition initPose{{0.0, 0.0, 0.0}, rtl::Quaternion<double>::identity(), 0};
        visualizationHandler_.updateOriginToRootTf(initPose);

        for (size_t i = 0; !dataLoader_.isOnEnd(); i++) {

            auto data = dataLoader_.getNextData();
            auto data_ts = data->getTimestamp();

            std::stringstream ss;
            ss << data_ts << " " << data->toString();
            context_.logger_.debug(ss.str());

            if(last_system_ts == 0) {
                last_system_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                last_data_ts = data_ts;
            }

            while(true) {
                auto system_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                if((data_ts - last_data_ts) < (system_ts - last_system_ts)*maxReplayerRate_) {
                    last_data_ts = data_ts;
                    last_system_ts = system_ts;
                    break;
                }
            }


            auto dataType = data->getType();
            auto sensorFrame = getFrameForData(data);
            auto sensorFailCheckID = failChecker_.frameToFailcheckID(sensorFrame);
            failChecker_.onNewData(data, sensorFailCheckID);
            auto sensorScore = failChecker_.getSensorStatus(sensorFailCheckID);

            if(sensorScore < 0.9) {
                context_.logger_.warning("Sensor Score is too low");
                continue;
            }

            /* ... data processing ... */

            if(dataType == DataModels::DataModelTypes::kCameraDataModelType) {

                processRGBCameraData(data, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kCameraIrDataModelType) {

                processIRCameraData(data, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kGnssPositionDataModelType) {

                processGnssPoseData(data, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kGnssTimeDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuDquatDataModelType) {

                processImuDQuatData(data, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kImuGnssDataModelType) {

                processImuGnssData(data, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kImuImuDataModelType) {

                processImuImuData(data, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kImuMagDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuPressDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuTempDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuTimeDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kLidarScanDataModelType) {

                processLidarScanData(data, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kGenericDataModelType) {
                context_.logger_.warning("Received Generic data model from DataLoader");
            } else if (dataType == DataModels::DataModelTypes::kErrorDataModelType) {
                context_.logger_.warning("Received Error data model from DataLoader");
            } else {
                context_.logger_.warning("Unepected type of data model from DataLoader");
            }
        }
    }


    void MapBuilder::clearData() {
        dataLoader_.clear();
    }


    void MapBuilder::processRGBCameraData(std::shared_ptr<DataModels::GenericDataModel> data, std::string& sensorFrame) {
        static int cnt = 0;

        auto imgData = std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(data);
        auto batches = pointCloudAggregator_.getAllBatches();
        depthMap_.updatePointcloudData(batches);


        auto detections3D = depthMap_.onNewCameraData(imgData, selfModel_.getPosition());
        auto frustums = detectionProcessor_.onNew3DYoloDetections(detections3D, sensorFrame);

        if(sensorFrame == LocalMap::Frames::kCameraLeftFront) {

            auto imuToCameraTf = context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kCameraIr);
            auto reprojectedYoloDetections = yoloIrReprojector_.onNewDetections(frustums, imuToCameraTf.inverted());

            if(!reprojectedYoloDetections->empty()) {
                auto imgWidthHeight = yoloIrReprojector_.getLastIrFrameWidthHeight();
                yoloIRDetectionWriter_.writeDetections(reprojectedYoloDetections,
                                                       yoloIrReprojector_.getCurrentIrFrameNo());
                yoloIRDetectionWriter_.writeDetectionsAsTrainData(reprojectedYoloDetections,
                                                                  yoloIrReprojector_.getCurrentIrFrameNo(), imgWidthHeight.first, imgWidthHeight.second);
                yoloIRDetectionWriter_.writeIRImageAsTrainData(yoloIrReprojector_.getLastIrFrame(),
                                                               yoloIrReprojector_.getCurrentIrFrameNo());
            }
        }

        localMap_.setFrustumDetections(frustums, sensorFrame);
        visualizationHandler_.drawFrustumDetections(localMap_.getFrustumDetections());

        if(cnt++ >= 3) {
            auto aggregatedPointcloud = pointCloudAggregator_.getAggregatedPointCloud();
            auto downsampledAggregatedPc = pointCloudProcessor_.downsamplePointCloud(aggregatedPointcloud);
            //globalPointcloudStorage_.addMorePointsToGlobalStorage(downsampledAggregatedPc);


            auto tunel = pointCloudAggregator_.getPointcloudCutout(pointCloudProcessor_.transformPointcloud(downsampledAggregatedPc, selfModel_.getPosition().toTf().inverted()),
                                                                   rtl::BoundingBox3D<float>{rtl::Vector3D<float>{-30.0f, -10.0f, -0.5f},
                                                                                             rtl::Vector3D<float>{ 30.0f,  10.0f, 10.0f}});
            auto downsampledTunel = pointCloudProcessor_.downsamplePointCloud(tunel);
            auto lidarObstacles = lidarObjectDetector_.detectObstacles(downsampledTunel);
            localMap_.setLidarDetections(
                    objectAggregator_.aggregateLidarDetections(localMap_.getLidarDetections(),
                                                               lidarObstacles));


            visualizationHandler_.drawAggregatedPointcloud(downsampledAggregatedPc);
            visualizationHandler_.drawLidarDetection(lidarObstacles);
            visualizationHandler_.drawPointcloudCutout(tunel);
            visualizationHandler_.drawLidarDetection(localMap_.getLidarDetections());
        }

        visualizationHandler_.drawRGBImage(imgData);
    }


    void MapBuilder::processIRCameraData(std::shared_ptr<DataModels::GenericDataModel> data, std::string& sensorFrame) {
        auto irCameraFrame = std::dynamic_pointer_cast<DataModels::CameraIrFrameDataModel>(data);
        static size_t frameCnt = 0;

        auto originToImuTf = selfModel_.getPosition().toTf();

        auto points2Dand3Dpair = depthMap_.getPointsInCameraFoV(
                irCameraFrame->getCameraIdentifier(),
                irCameraFrame->getImage().cols,
                irCameraFrame->getImage().rows,
                originToImuTf,
                false);
        auto img = lidarIrImgPlotter_.renderLidarPointsToImg(points2Dand3Dpair->first,
                                                             points2Dand3Dpair->second,
                                                             irCameraFrame->getImage().cols,
                                                             irCameraFrame->getImage().rows);
        lidarIrImgPlotter_.saveImage(img, frameCnt);
        yoloIrReprojector_.onNewIRFrame(irCameraFrame);
        visualizationHandler_.drawIRImage(irCameraFrame);
        frameCnt++;
    }


    void MapBuilder::processGnssPoseData(std::shared_ptr<DataModels::GenericDataModel> data, std::string& sensorFrame) {
        auto poseData = std::dynamic_pointer_cast<DataModels::GnssPoseDataModel>(data);
        gnssPoseLogger_.onGnssPose(poseData);
        selfModel_.onGnssPose(poseData);

        auto currentPose = selfModel_.getPosition();
        imuPoseLogger_.setAltitude(currentPose.getPosition().z());

        visualizationHandler_.updateOriginToRootTf(currentPose);
        visualizationHandler_.drawRawGnssTrajectory(gnssPoseLogger_.getPositionHistory());
        visualizationHandler_.drawFilteredTrajectory(selfModel_.getPositionHistory());
        visualizationHandler_.drawGnssPoseData(poseData);
        visualizationHandler_.drawTelemetry(selfModel_.getTelemetryString());
        visualizationHandler_.drawImuAvgData(selfModel_.getAvgAcceleration());
        visualizationHandler_.drawVelocityData(selfModel_.getSpeedVector());
    }


    void MapBuilder::processImuDQuatData(std::shared_ptr<DataModels::GenericDataModel> data, std::string& sensorFrame) {
        auto dQuatData = std::dynamic_pointer_cast<DataModels::ImuDquatDataModel>(data);
        selfModel_.onImuDquatData(dQuatData);
    }


    void MapBuilder::processImuGnssData(std::shared_ptr<DataModels::GenericDataModel> data, std::string& sensorFrame) {
        auto poseData = std::dynamic_pointer_cast<DataModels::ImuGnssDataModel>(data);
        imuPoseLogger_.onImuGps(poseData);
        visualizationHandler_.drawImuGpsTrajectory(imuPoseLogger_.getPositionHistory());
    }


    void MapBuilder::processImuImuData(std::shared_ptr<DataModels::GenericDataModel> data, std::string& sensorFrame) {
        auto imuData = std::dynamic_pointer_cast<DataModels::ImuImuDataModel>(data);

        imuProcessor_.setOrientation(imuData->getOrientation());
        auto linAccNoGrav = imuProcessor_.removeGravitaionAcceleration(imuData->getLinearAcc());

        selfModel_.onImuImuData(imuData);
        visualizationHandler_.drawImuData(linAccNoGrav);
    }


    void MapBuilder::processLidarScanData(std::shared_ptr<DataModels::GenericDataModel> data, std::string& sensorFrame) {
        auto lidarData = std::dynamic_pointer_cast<DataModels::LidarScanDataModel>(data);
        lidarData->registerFilter(std::bind(&Algorithms::LidarFilter::applyFiltersOnLidarData, &lidarFilter_, std::placeholders::_1));

        auto lidarID = lidarData->getLidarIdentifier();

        if (lidarDataHistory_.count(sensorFrame) > 0) {

            auto lidarTF = context_.tfTree_.getTransformationForFrame(sensorFrame);
            auto lastLidarTimestamp = (lidarDataHistory_[sensorFrame])->getTimestamp();
            auto poseBefore = selfModel_.estimatePositionInTime( lastLidarTimestamp );
            auto poseNow = selfModel_.getPosition();
            auto poseDiff = poseNow-poseBefore;

            auto downsampledScan = pointCloudProcessor_.downsamplePointCloud(lidarData->getScan());
            auto batches = pointCloudExtrapolator_.splitPointCloudToBatches(downsampledScan, poseBefore, poseDiff, lidarTF);

            pointCloudAggregator_.filterOutBatches(lidarData->getTimestamp());
            pointCloudAggregator_.addPointCloudBatches(batches);


            if(lidarID == DataLoader::LidarIdentifier::kLeftLidar) {

                size_t laserNo = 5;
                leftLidarLaserAggregator_.onNewLaserData(lidarData->getRawScan(), poseBefore, poseDiff, lidarTF);
                auto aggregatedLaser = leftLidarLaserAggregator_.getAggregatedLaser(laserNo);
                auto laserApproximation = leftLaserSegmenter_.getApproximation(laserNo);

                leftLaserSegmenter_.clear();
                for( size_t laserNo = 0 ; laserNo < 32 ; laserNo++) {
                    leftLaserSegmenter_.onLaserData(leftLidarLaserAggregator_.getAggregatedLaser(laserNo), laserNo);
                }

                visualizationHandler_.drawAggregatedLasers(aggregatedLaser);

            } else if (lidarID == DataLoader::LidarIdentifier::kRightLidar) {
                rightLidarLaserAggregator_.onNewLaserData(lidarData->getRawScan(), poseBefore, poseDiff, lidarTF);
                rightLaserSegmenter_.clear();
                for( size_t laserNo = 0 ; laserNo < 32 ; laserNo++) {
                    rightLaserSegmenter_.onLaserData(rightLidarLaserAggregator_.getAggregatedLaser(laserNo), laserNo);
                }
            } else {
                context_.logger_.warning("Unexpected lidar identifier");
            }

            auto approximations = std::make_shared<std::vector<rtl::LineSegment3D<double>>>();
            auto roadApproximations = std::make_shared<std::vector<rtl::LineSegment3D<double>>>();

            for (auto& approximation : *leftLaserSegmenter_.getAllApproximations()) { approximations->push_back(approximation); }
            for (auto& approximation : *rightLaserSegmenter_.getAllApproximations()) { approximations->push_back(approximation); }

            for (auto& road : *leftLaserSegmenter_.getAllRoads()) { roadApproximations->push_back(road); }
            for (auto& road : *rightLaserSegmenter_.getAllRoads()) { roadApproximations->push_back(road); }

            visualizationHandler_.drawLidarApproximations(approximations);
            visualizationHandler_.drawLidarApproximationsRoad(roadApproximations);
        }
        lidarDataHistory_[sensorFrame] = lidarData;
        visualizationHandler_.drawLidarData(lidarData);
        visualizationHandler_.drawSelf();
    }


    std::string MapBuilder::getFrameForData(std::shared_ptr<DataModels::GenericDataModel> data) {
        auto type = data->getType();


        switch (type) {

            case DataModels::DataModelTypes::kCameraDataModelType:
                switch(std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(data)->getCameraIdentifier()) {
                    case DataLoader::CameraIndentifier::kCameraLeftFront:
                        return LocalMap::Frames::kCameraLeftFront;
                    case DataLoader::CameraIndentifier::kCameraLeftSide:
                        return LocalMap::Frames::kCameraLeftSide;
                    case DataLoader::CameraIndentifier::kCameraRightFront:
                        return LocalMap::Frames::kCameraRightFront;
                    case DataLoader::CameraIndentifier::kCameraRightSide:
                        return LocalMap::Frames::kCameraRightSide;
                    default:
                        context_.logger_.error("Unable to estimate camera frame!");
                        return LocalMap::Frames::kErr;
                }

            case DataModels::DataModelTypes::kCameraIrDataModelType:
                return LocalMap::Frames::kCameraIr;

            case DataModels::DataModelTypes::kLidarScanDataModelType:
                switch(std::dynamic_pointer_cast<DataModels::LidarScanDataModel>(data)->getLidarIdentifier()) {
                    case DataLoader::LidarIdentifier::kLeftLidar:
                        return LocalMap::Frames::kLidarLeft;
                    case DataLoader::LidarIdentifier::kRightLidar:
                        return LocalMap::Frames::kLidarRight;
                    default:
                        context_.logger_.error("Unable to estimate lidar frame!");
                        return LocalMap::Frames::kErr;
                }

            case DataModels::DataModelTypes::kImuDquatDataModelType:
            case DataModels::DataModelTypes::kImuGnssDataModelType:
            case DataModels::DataModelTypes::kImuImuDataModelType:
            case DataModels::DataModelTypes::kImuMagDataModelType:
            case DataModels::DataModelTypes::kImuPressDataModelType:
            case DataModels::DataModelTypes::kImuTimeDataModelType:
            case DataModels::DataModelTypes::kImuTempDataModelType:
                return LocalMap::Frames::kImuFrame;

            case DataModels::DataModelTypes::kGnssPositionDataModelType:
            case DataModels::DataModelTypes::kGnssTimeDataModelType:
                return LocalMap::Frames::kGnssAntennaRear;

            default:
                context_.logger_.error("Unable to estimate sensor frame!");
                return LocalMap::Frames::kErr;
        }
    }
}