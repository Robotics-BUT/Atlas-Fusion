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
#include "MapBuilderMacros.h"
#include "RustStyle.h"

#include "data_models/all.h"
#include "data_models/DataModelTypes.h"

#include "algorithms/Projector.h"
#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive {


    void MapBuilder::loadData() {
        Timer timer("Data loading");
        // Load all data through all available data loaders
        dataLoader_.loadData(destinationFolder_);
        std::cout << "Total No. of loaded data: " << dataLoader_.getDataSize() << std::endl;

        std::vector<std::string> cameraFrames = {
                LocalMap::Frames::kCameraLeftFront,
                LocalMap::Frames::kCameraLeftSide,
                LocalMap::Frames::kCameraRightFront,
                LocalMap::Frames::kCameraRightSide,
                LocalMap::Frames::kCameraIr,
        };

        // Additionally propagate the individual camera calibration parameters
        for (let &frame: cameraFrames) {
            mut cameraID = dataLoader_.getCameraIDfromFrame(frame);
            mut params = dataLoader_.getCameraCalibDataForCameraID(cameraID);
            mut tf = context_.tfTree_.getTransformationForFrame(frame);

            if (params.getType() == DataModels::DataModelTypes::kCameraCalibrationParamsDataModelType) {
                visualizationHandler_.setCameraCalibParamsForCameraId(params, frame);

                mut projector = std::make_shared<Algorithms::Projector>(params.getMatIntrinsicParams(), params.getMatDistortionParams(), tf);
                if (frame == LocalMap::Frames::kCameraIr) {
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

        while (!dataLoader_.isOnEnd()) {
            mut data = dataLoader_.getNextData();
            mut data_ts = data->getTimestamp();

            mut dataType = data->getType();
            // Timer timer("Data frame of type " + std::to_string((int) dataType) + " processing");

            std::stringstream ss;
            ss << data_ts << " " << data->toString();
            context_.logger_.debug(ss.str());

            if (last_system_ts == 0) {
                last_system_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                last_data_ts = data_ts;
            }

            while (true) {
                mut system_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                if ((data_ts - last_data_ts) < (system_ts - last_system_ts) * (uint64_t) maxReplayerRate_) {
                    last_data_ts = data_ts;
                    last_system_ts = system_ts;
                    break;
                }
            }

            mut sensorFrame = frameTypeFromDataModel(data);
            mut sensorFailCheckID = failChecker_.frameToFailcheckID(sensorFrame);
            failChecker_.onNewData(data, sensorFailCheckID);
            mut sensorScore = failChecker_.getSensorStatus(sensorFailCheckID);

            if (sensorScore < 0.9) {
                context_.logger_.warning("Sensor Score is too low");
                continue;
            }

            /* ... data processing ... */
            if (dataType == DataModels::DataModelTypes::kCameraDataModelType) {

                mut imgData = std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(data);
                processRGBCameraData(imgData, sensorFrame);
                cache_.setNewRGBFrame(imgData);

            } else if (dataType == DataModels::DataModelTypes::kCameraIrDataModelType) {

                mut irCameraFrame = std::dynamic_pointer_cast<DataModels::CameraIrFrameDataModel>(data);
                processIRCameraData(irCameraFrame);
                cache_.setNewIRFrame(irCameraFrame);

            } else if (dataType == DataModels::DataModelTypes::kGnssPositionDataModelType) {

                mut poseData = std::dynamic_pointer_cast<DataModels::GnssPoseDataModel>(data);
                processGnssPoseData(poseData);

            } else if (dataType == DataModels::DataModelTypes::kGnssTimeDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuDquatDataModelType) {

                mut dQuatData = std::dynamic_pointer_cast<DataModels::ImuDquatDataModel>(data);
                processImuDQuatData(dQuatData);

            } else if (dataType == DataModels::DataModelTypes::kImuGnssDataModelType) {

                mut poseData = std::dynamic_pointer_cast<DataModels::ImuGnssDataModel>(data);
                processImuGnssData(poseData);

            } else if (dataType == DataModels::DataModelTypes::kImuImuDataModelType) {

                mut imuData = std::dynamic_pointer_cast<DataModels::ImuImuDataModel>(data);
                processImuImuData(imuData);

            } else if (dataType == DataModels::DataModelTypes::kImuMagDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuPressDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuTempDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuTimeDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kLidarScanDataModelType) {

                mut lidarData = std::dynamic_pointer_cast<DataModels::LidarScanDataModel>(data);
                lidarData->registerFilter(std::bind(&Algorithms::LidarFilter::applyFiltersOnLidarData, &lidarFilter_,
                                                    std::placeholders::_1));
                processLidarScanData(lidarData);
                cache_.setNewLidarScan(lidarData);

            } else if (dataType == DataModels::DataModelTypes::kRadarTiScanDataModelType) {

                let radarData = std::dynamic_pointer_cast<DataModels::RadarTiDataModel>(data);
                processRadarTiData(radarData);

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


    void MapBuilder::processRGBCameraData(const std::shared_ptr<DataModels::CameraFrameDataModel> &imgData, const std::string &sensorFrame) {
        static int cnt = 0;
        mut batches = pointCloudAggregator_.getAllBatches();
        depthMap_.updatePointCloudData(batches);

        mut detections3D = depthMap_.onNewCameraData(imgData, selfModel_.getPosition());
        mut frustums = detectionProcessor_.onNew3DYoloDetections(detections3D, sensorFrame);

        if (RGB_Detection_To_IR_Projection) {
            if (sensorFrame == LocalMap::Frames::kCameraLeftFront) {

                mut imuToCameraTf = context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kCameraIr);
                mut reprojectedYoloDetections = yoloIrReprojector_.onNewDetections(frustums, imuToCameraTf.inverted());

                if (!reprojectedYoloDetections.empty()) {
                    mut imgWidthHeight = yoloIrReprojector_.getLastIrFrameWidthHeight();
                    yoloIRDetectionWriter_.writeDetections(reprojectedYoloDetections, yoloIrReprojector_.getCurrentIrFrameNo());
                    yoloIRDetectionWriter_.writeDetectionsAsTrainData(reprojectedYoloDetections, yoloIrReprojector_.getCurrentIrFrameNo(), imgWidthHeight.first,
                                                                      imgWidthHeight.second);
                    yoloIRDetectionWriter_.writeIRImageAsTrainData(yoloIrReprojector_.getLastIrFrame(), yoloIrReprojector_.getCurrentIrFrameNo());
                }


                if (Depth_Map_For_IR) {
                    try {
                        mut irCameraFrame = DataModels::CameraIrFrameDataModel(imgData->getTimestamp(), imgData->getImage(), 0.0f, 100.0f, imgData->getCameraIdentifier(),
                                                                               std::vector<DataModels::YoloDetection>{});
                        static size_t frameCnt = 0;

                        mut originToImuTf = selfModel_.getPosition().toTf();

                        mut points2Dand3Dpair = depthMap_.getPointsInCameraFoV(irCameraFrame.getCameraIdentifier(), irCameraFrame.getImage().cols, irCameraFrame.getImage().rows,
                                                                               originToImuTf, false);
                        mut img = lidarIrImgPlotter_.renderLidarPointsToImg(points2Dand3Dpair->first, points2Dand3Dpair->second, irCameraFrame.getImage().cols,
                                                                            irCameraFrame.getImage().rows, 3);

                        if (yoloIrReprojector_.getCurrentIrFrameNo() > 0) {
                            lidarIrImgPlotter_.saveImage(img, frameCnt, "_depth", "png");
                            mut rgb_cutout = simpleImageProcessor_.convertLeftFrontRGBToIrFieldOfView(
                                    imgData->getImage());
                            lidarIrImgPlotter_.saveImage(std::make_shared<cv::Mat>(rgb_cutout), frameCnt,
                                                         "_rgb", "jpeg");
                            lidarIrImgPlotter_.saveImage(
                                    std::make_shared<cv::Mat>(yoloIrReprojector_.getLastIrFrame()->getImage()),
                                    frameCnt,
                                    "_ir",
                                    "jpeg");
                            frameCnt++;
                        }
                    }
                    catch (std::exception &e) {
                        std::cerr << e.what() << std::endl;
                    }
                }
            }
        }

        localMap_.setFrustumDetections(frustums, sensorFrame);
        visualizationHandler_.drawFrustumDetections(localMap_.getFrustumDetections());

        if (cnt++ >= 3) {

            cnt = 0;
            mut aggregatedPointCloud = pointCloudAggregator_.getAggregatedPointCloud();
            mut downSampledAggregatedPc = pointCloudProcessor_.downsamplePointCloud(aggregatedPointCloud);

            //mut tunel = pointCloudAggregator_.getPointCloudCutout(pointCloudProcessor_.transformPointCloud(downSampledAggregatedPc, selfModel_.getPosition().toTf().inverted()),
            //                                                      rtl::BoundingBox3D<float>{rtl::Vector3D<float>{-30.0f, -10.0f, -0.5f},
            //                                                                                rtl::Vector3D<float>{30.0f, 10.0f, 10.0f}});
            //mut downSampledTunel = pointCloudProcessor_.downsamplePointCloud(tunel);
            //mut lidarObstacles = lidarObjectDetector_.detectObstacles(downSampledTunel);
            //localMap_.setLidarDetections(objectAggregator_.aggregateLidarDetections(localMap_.getLidarDetections(), lidarObstacles));

            visualizationHandler_.drawAggregatedPointcloud(downSampledAggregatedPc);
            //visualizationHandler_.drawLidarDetection(lidarObstacles);
            //visualizationHandler_.drawPointcloudCutout(tunel);
            //visualizationHandler_.drawLidarDetection(localMap_.getLidarDetections());
        }

        visualizationHandler_.drawRGBImage(imgData);
    }


    void MapBuilder::processIRCameraData(const std::shared_ptr<DataModels::CameraIrFrameDataModel> &irCameraFrame) {

        mut originToImuTf = selfModel_.getPosition().toTf();
        mut points2Dand3Dpair = depthMap_.getPointsInCameraFoV(
                irCameraFrame->getCameraIdentifier(),
                irCameraFrame->getImage().cols,
                irCameraFrame->getImage().rows,
                originToImuTf,
                false);
        mut img = lidarIrImgPlotter_.renderLidarPointsToImg(points2Dand3Dpair->first,
                                                            points2Dand3Dpair->second,
                                                            irCameraFrame->getImage().cols,
                                                            irCameraFrame->getImage().rows);
        yoloIrReprojector_.onNewIRFrame(irCameraFrame);
        visualizationHandler_.drawIRImage(irCameraFrame);
    }


    void
    MapBuilder::processGnssPoseData(const std::shared_ptr<DataModels::GnssPoseDataModel> &poseData) {

        gnssPoseLogger_.onGnssPose(poseData);
        selfModel_.onGnssPose(poseData);

        mut currentPose = selfModel_.getPosition();
        imuPoseLogger_.setAltitude(currentPose.getPosition().z());

        visualizationHandler_.updateOriginToRootTf(currentPose);
        visualizationHandler_.drawRawGnssTrajectory(gnssPoseLogger_.getPositionHistory());
        visualizationHandler_.drawFilteredTrajectory(selfModel_.getPositionHistory());
        visualizationHandler_.drawGnssPoseData(poseData);
        visualizationHandler_.drawTelemetry(selfModel_.getTelemetryString());
        visualizationHandler_.drawImuAvgData(selfModel_.getAvgAcceleration());
        visualizationHandler_.drawVelocityData(selfModel_.getSpeedVector());
    }


    void MapBuilder::processImuDQuatData(const std::shared_ptr<DataModels::ImuDquatDataModel> &dQuatData) {
        selfModel_.onImuDquatData(dQuatData);
    }


    void
    MapBuilder::processImuGnssData(const std::shared_ptr<DataModels::ImuGnssDataModel> &poseData) {
        imuPoseLogger_.onImuGps(poseData);
        visualizationHandler_.drawImuGpsTrajectory(imuPoseLogger_.getPositionHistory());
    }


    void MapBuilder::processImuImuData(const std::shared_ptr<DataModels::ImuImuDataModel> &imuData) {

        imuProcessor_.setOrientation(imuData->getOrientation());
        mut linAccNoGrav = imuProcessor_.removeGravitaionAcceleration(imuData->getLinearAcc());

        selfModel_.onImuImuData(imuData);
        visualizationHandler_.drawImuData(linAccNoGrav);
    }


    void MapBuilder::processLidarScanData(const std::shared_ptr<DataModels::LidarScanDataModel> &lidarData) {
        let lidarID = lidarData->getLidarIdentifier();
        if (cache_.getLidarScan(lidarID) != nullptr) {
            if (Short_Term_Lidar_Aggregation) {
                aggregateLidar(lidarData);
            }
            if (Lidar_Laser_Approx_And_Seg) {
                approximateLidar(lidarData);
            }
        }
        visualizationHandler_.drawLidarData(lidarData);
        visualizationHandler_.drawSelf();
    }

    void MapBuilder::processRadarTiData(const std::shared_ptr<DataModels::RadarTiDataModel> &data) {
        let radarData = std::dynamic_pointer_cast<DataModels::RadarTiDataModel>(data);
        visualizationHandler_.drawRadarTiObjects(radarData->getObjects());
    }

    void MapBuilder::aggregateLidar(const std::shared_ptr<DataModels::LidarScanDataModel> &lidarData) {
        Timer tim("Point cloud aggregation");
        let lidarID = lidarData->getLidarIdentifier();
        let sensorFrame = frameTypeFromDataModel(lidarData);
        mut lidarTF = context_.tfTree_.getTransformationForFrame(sensorFrame);
        let scan = cache_.getLidarScan(lidarID);
        let lastLidarTimestamp = scan->getTimestamp();

        mut poseBefore = selfModel_.estimatePositionInTime(lastLidarTimestamp);
        mut poseNow = selfModel_.getPosition();
        mut poseDiff = poseNow - poseBefore;

        mut downsampledScan = pointCloudProcessor_.downsamplePointCloud(lidarData->getScan());
        mut batches = pointCloudExtrapolator_.splitPointCloudToBatches(downsampledScan, poseBefore, poseDiff, lidarTF);
        pointCloudAggregator_.filterOutBatches(lidarData->getTimestamp());
        pointCloudAggregator_.addPointCloudBatches(batches);

        if (Global_Lidar_Aggregation) {
            mut aggregatedPointcloud = pointCloudAggregator_.getAggregatedPointCloud();
            mut downsampledAggregatedPc = pointCloudProcessor_.downsamplePointCloud(aggregatedPointcloud);
            globalPointcloudStorage_.addMorePointsToGlobalStorage(downsampledAggregatedPc);
            visualizationHandler_.drawGlobalPointcloud(globalPointcloudStorage_.getEntirePointcloud());
        }
    }

    void MapBuilder::approximateLidar(const std::shared_ptr<DataModels::LidarScanDataModel> &lidarData) {
        let lidarID = lidarData->getLidarIdentifier();
        let sensorFrame = frameTypeFromDataModel(lidarData);
        mut lidarTF = context_.tfTree_.getTransformationForFrame(sensorFrame);
        let scan = cache_.getLidarScan(lidarID);
        let lastLidarTimestamp = scan->getTimestamp();

        mut poseBefore = selfModel_.estimatePositionInTime(lastLidarTimestamp);
        mut poseNow = selfModel_.getPosition();
        mut poseDiff = poseNow - poseBefore;

        if (lidarID == DataLoader::LidarIdentifier::kLeftLidar) {

            size_t laserNo = 5;
            leftLidarLaserAggregator_.onNewLaserData(lidarData->getRawScan(), poseBefore, poseDiff, lidarTF);
            mut aggregatedLaser = leftLidarLaserAggregator_.getAggregatedLaser(laserNo);
            mut laserApproximation = leftLaserSegmenter_.getApproximation(laserNo);

            leftLaserSegmenter_.clear();
            for (size_t i = 0; i < 32; i++) {
                leftLaserSegmenter_.onLaserData(leftLidarLaserAggregator_.getAggregatedLaser(i), i);
            }
            visualizationHandler_.drawAggregatedLasers(aggregatedLaser);

        } else if (lidarID == DataLoader::LidarIdentifier::kRightLidar) {
            rightLidarLaserAggregator_.onNewLaserData(lidarData->getRawScan(), poseBefore, poseDiff, lidarTF);
            rightLaserSegmenter_.clear();
            for (size_t laserNo = 0; laserNo < 32; laserNo++) {
                rightLaserSegmenter_.onLaserData(rightLidarLaserAggregator_.getAggregatedLaser(laserNo),
                                                 laserNo);
            }
        } else if (lidarID == DataLoader::LidarIdentifier::kCenterLidar) {
            // Central lidar provides incompatible data.
        } else {
            context_.logger_.warning("Unexpected lidar identifier");
        }

        mut approximations = std::make_shared<std::vector<rtl::LineSegment3D<double>>>();
        mut roadApproximations = std::make_shared<std::vector<rtl::LineSegment3D<double>>>();

        for (mut &approximation: *leftLaserSegmenter_.getAllApproximations()) {
            approximations->push_back(approximation);
        }
        for (mut &approximation: *rightLaserSegmenter_.getAllApproximations()) {
            approximations->push_back(approximation);
        }

        for (mut &road: *leftLaserSegmenter_.getAllRoads()) { roadApproximations->push_back(road); }
        for (mut &road: *rightLaserSegmenter_.getAllRoads()) { roadApproximations->push_back(road); }

        visualizationHandler_.drawLidarApproximations(approximations);
        visualizationHandler_.drawLidarApproximationsRoad(roadApproximations);

    }
}