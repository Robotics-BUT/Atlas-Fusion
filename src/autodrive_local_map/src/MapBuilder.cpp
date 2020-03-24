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
                depthMap_.addProjector(projector, cameraID);
                detectionProcessor_.addProjector(projector, frame);
            } else {
                context_.logger_.warning("Unable to read camera calib data");
            }
        }
    }

    void MapBuilder::buildMap() {

        uint64_t dataCounter = 0;
        int64_t last_system_ts = 0;
        uint64_t last_data_ts = 0;

        DataModels::LocalPosition initPose{{},{},0};
        visualizationHandler_.updateOriginToRootTf(initPose);

        for (size_t i = 0; !dataLoader_.isOnEnd(); i++) {

            auto data = dataLoader_.getNextData();
            auto data_ts = data->getTimestamp();

//            static uint64_t timeBegin = data->getTimestamp();
//            uint64_t diff = 0;
//            do {
//                diff = (data->getTimestamp() - timeBegin );
//                data = dataLoader_.getNextData();
//            } while(diff < 10e9);



//            if(dataCounter++ > 1000) {
//                return;
//            }

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

                static int cnt = 0;
                static int globalPcCnt = 0;

                auto imgData = std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(data);
                auto batches = pointCloudAggregator_.getAllBatches();
                depthMap_.updatePointcloudData(batches);


                auto detections3D = depthMap_.onNewCameraData(imgData, selfModel_.getPosition());
                auto frustums = detectionProcessor_.onNew3DYoloDetections(detections3D, sensorFrame);


                localMap_.setFrustumDetections(frustums, sensorFrame);
                visualizationHandler_.drawFrustumDetections(localMap_.getFrustumDetections());


                if(cnt++ >= 3) {
                    auto aggregatedPointcloud = pointCloudAggregator_.getAggregatedPointCloud();
                    auto downsampledAggregatedPc = pointCloudProcessor_.downsamplePointCloud(aggregatedPointcloud);
                    globalPointcloudStorage_.addMorePointsToGlobalStorage(downsampledAggregatedPc);


                    auto tunel = pointCloudAggregator_.getPointcloudCutout(pointCloudProcessor_.transformPointcloud(downsampledAggregatedPc, selfModel_.getPosition().toTf().inverted()),
                                                                           rtl::BoundingBox3f{rtl::Vector3Df{-30.0f, -10.0f, -0.5f},
                                                                                              rtl::Vector3Df{ 30.0f,  10.0f, 10.0f}});
                    auto downsampledTunel = pointCloudProcessor_.downsamplePointCloud(tunel);
                    auto lidarObstacles = lidarObjectDetector_.detectObstacles(downsampledTunel);
                    localMap_.setLidarDetections(
                            objectAggregator_.aggregateLidarDetections(localMap_.getLidarDetections(),
                                                                       lidarObstacles));


                    visualizationHandler_.drawAggregatedPointcloud(downsampledAggregatedPc);
                    visualizationHandler_.drawLidarDetection(lidarObstacles);
                    visualizationHandler_.drawPointcloudCutout(tunel);
                    visualizationHandler_.drawLidarDetection(localMap_.getLidarDetections());

                    if(globalPcCnt++ >= 100) {
                        //visualizationHandler_.drawGlobalPointcloud(globalPointcloudStorage_.getEntirePointcloud());
                        globalPcCnt = 0;
                    }
                    cnt = 0;
                }

                visualizationHandler_.drawRGBImage(imgData);

            } else if (dataType == DataModels::DataModelTypes::kCameraIrDataModelType) {

                auto irCameraFrame = std::dynamic_pointer_cast<DataModels::CameraIrFrameDataModel>(data);
                visualizationHandler_.drawIRImage(irCameraFrame);

            } else if (dataType == DataModels::DataModelTypes::kGnssPositionDataModelType) {

                auto poseData = std::dynamic_pointer_cast<DataModels::GnssPoseDataModel>(data);
                gnssPoseLogger_.onGnssPose(poseData);
                selfModel_.onGnssPose(poseData);

                //auto currentPose = poseLogger_.getPosition();
                auto currentPose = selfModel_.getPosition();
                imuPoseLogger_.setAltitude(currentPose.getPosition().z());

                visualizationHandler_.updateOriginToRootTf(currentPose);
                visualizationHandler_.drawRawGnssTrajectory(gnssPoseLogger_.getPositionHistory());
                visualizationHandler_.drawFilteredTrajectory(selfModel_.getPositionHistory());
                visualizationHandler_.drawGnssPoseData(poseData);
                visualizationHandler_.drawTelemetry(selfModel_.getTelemetryString());
                visualizationHandler_.drawImuAvgData(selfModel_.getAvgAcceleration());
                visualizationHandler_.drawSpeedData(selfModel_.getSpeedVector());

            } else if (dataType == DataModels::DataModelTypes::kGnssTimeDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuDquatDataModelType) {

                auto dQuatData = std::dynamic_pointer_cast<DataModels::ImuDquatDataModel>(data);
                selfModel_.onImuDquatData(dQuatData);

            } else if (dataType == DataModels::DataModelTypes::kImuGnssDataModelType) {

                auto poseData = std::dynamic_pointer_cast<DataModels::ImuGnssDataModel>(data);
                imuPoseLogger_.onImuGps(poseData);
                visualizationHandler_.drawImuGpsTrajectory(imuPoseLogger_.getPositionHistory());

            } else if (dataType == DataModels::DataModelTypes::kImuImuDataModelType) {

                auto imuData = std::dynamic_pointer_cast<DataModels::ImuImuDataModel>(data);

                imuProcessor_.setOrientation(imuData->getOrientation());
                auto linAccNoGrav = imuProcessor_.removeGravitaionAcceleration(imuData->getLinearAcc());

                selfModel_.onImuImuData(imuData);
                visualizationHandler_.drawImuData(linAccNoGrav);

            } else if (dataType == DataModels::DataModelTypes::kImuMagDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuPressDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuTempDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuTimeDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kLidarScanDataModelType) {

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
                        leftLidarLaserAggregator_.onNewLaserData(lidarData->getRawScan(), poseBefore, poseDiff, lidarTF);
                        visualizationHandler_.drawAggregatedLasers(leftLidarLaserAggregator_.getAggregatedLaser(5));
                    } else if (lidarID == DataLoader::LidarIdentifier::kRightLidar) {
                        rightLidarLaserAggregator_.onNewLaserData(lidarData->getRawScan(), poseBefore, poseDiff, lidarTF);
                    } else {
                        context_.logger_.warning("Unexpected lidar identifier");
                    }

                }
                lidarDataHistory_[sensorFrame] = lidarData;
                visualizationHandler_.drawLidarData(lidarData);
                visualizationHandler_.drawSelf();

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