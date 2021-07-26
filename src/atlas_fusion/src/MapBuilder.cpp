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
#include "algorithms/pointcloud/SphericalPoint.h"

namespace AtlasFusion {


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
                LocalMap::Frames::kCameraVirtual,
        };

        for(let& frame : cameraFrames) {
            mut cameraID = dataLoader_.getCameraIDfromFrame(frame);
            mut params = dataLoader_.getCameraCalibDataForCameraID(cameraID);
            mut tf = context_.tfTree_.getTransformationForFrame(frame);

            if(params->getType() == DataModels::DataModelTypes::kCameraCalibrationParamsDataModelType) {
                mut paramModel = std::dynamic_pointer_cast<DataModels::CameraCalibrationParamsDataModel>(params);
                visualizationHandler_.setCameraCalibParamsForCameraId(paramModel, frame);

                mut intrinsicParams = paramModel->getMatIntrinsicParams();
                mut distortionParams =  paramModel->getMatDistortionParams();

                mut projector = std::make_shared<Algorithms::Projector>(intrinsicParams, distortionParams, tf);
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

            mut data = dataLoader_.getNextData();
            mut data_ts = data->getTimestamp();

            std::stringstream ss;
            ss << data_ts << " " << data->toString();
            context_.logger_.debug(ss.str());

            if(last_system_ts == 0) {
                last_system_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                last_data_ts = data_ts;
            }

            while(true) {
                mut system_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                if((data_ts - last_data_ts) < (system_ts - last_system_ts)*maxReplayerRate_) {
                    last_data_ts = data_ts;
                    last_system_ts = system_ts;
                    break;
                }
            }


            mut dataType = data->getType();
            mut sensorFrame = getFrameForData(data);
            mut sensorFailCheckID = failChecker_.frameToFailcheckID(sensorFrame);
            failChecker_.onNewData(data, sensorFailCheckID);
            mut sensorScore = failChecker_.getSensorStatus(sensorFailCheckID);

            if(sensorScore < 0.9) {
                context_.logger_.warning("Sensor Score is too low");
                continue;
            }

            /* ... data processing ... */

            if(dataType == DataModels::DataModelTypes::kCameraDataModelType) {

                mut imgData = std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(data);
                processRGBCameraData(imgData, sensorFrame);
                cache_.setNewRGBFrame(imgData);

            } else if (dataType == DataModels::DataModelTypes::kCameraIrDataModelType) {

                mut irCameraFrame = std::dynamic_pointer_cast<DataModels::CameraIrFrameDataModel>(data);
                processIRCameraData(irCameraFrame, sensorFrame);
                cache_.setNewIRFrame(irCameraFrame);

            } else if (dataType == DataModels::DataModelTypes::kGnssPositionDataModelType) {

                mut poseData = std::dynamic_pointer_cast<DataModels::GnssPoseDataModel>(data);
                processGnssPoseData(poseData, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kGnssTimeDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuDquatDataModelType) {

                mut dQuatData = std::dynamic_pointer_cast<DataModels::ImuDquatDataModel>(data);
                processImuDQuatData(dQuatData, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kImuGnssDataModelType) {

                mut poseData = std::dynamic_pointer_cast<DataModels::ImuGnssDataModel>(data);
                processImuGnssData(poseData, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kImuImuDataModelType) {

                mut imuData = std::dynamic_pointer_cast<DataModels::ImuImuDataModel>(data);
                processImuImuData(imuData, sensorFrame);

            } else if (dataType == DataModels::DataModelTypes::kImuMagDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuPressDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuTempDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kImuTimeDataModelType) {

            } else if (dataType == DataModels::DataModelTypes::kLidarScanDataModelType) {

                mut lidarData = std::dynamic_pointer_cast<DataModels::LidarScanDataModel>(data);

                let lidarID = lidarData->getLidarIdentifier();
                if (Center_Lidar_Only && lidarID != DataLoader::LidarIdentifier::kCenterLidar) { continue; }

                lidarData->registerFilter(std::bind(&Algorithms::LidarFilter::applyFiltersOnLidarData, &lidarFilter_, std::placeholders::_1));
                processLidarScanData(lidarData, sensorFrame);
                cache_.setNewLidarScan(lidarData);

            } else if (dataType == DataModels::DataModelTypes::kRadarTiScanDataModelType) {

                let radarData = std::dynamic_pointer_cast<DataModels::RadarTiDataModel>(data);
                processRadarTiData(radarData, sensorFrame);

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


    void MapBuilder::processRGBCameraData(std::shared_ptr<DataModels::CameraFrameDataModel> imgData, std::string& sensorFrame) {

        static int cnt = 0;
        mut batches = pointCloudAggregator_.getAllBatches();
        depthMap_.updatePointcloudData(batches);

        if (imgData->getCameraIdentifier() == DataLoader::CameraIndentifier::kCameraLeftFront) {

            generateDepthMapForLastIR(imgData);
            generateDepthMapForRGBFrame(imgData);

            mut detections3D = depthMap_.onNewCameraData(imgData, selfModel_.getPosition());
            let frustums = detectionProcessor_.onNew3DYoloDetections(detections3D, sensorFrame);

            projectRGBDetectionsToIR(imgData, frustums);

            localMap_.setFrustumDetections(frustums, sensorFrame);
            visualizationHandler_.drawFrustumDetections(localMap_.getFrustumDetections());
        }



        if(cnt++ >= 3) {
            cnt = 0;
            mut aggregatedPointcloud = pointCloudAggregator_.getAggregatedPointCloud();
            mut downsampledAggregatedPc = pointCloudProcessor_.downsamplePointCloud(aggregatedPointcloud);

            mut tunel = pointCloudAggregator_.getPointcloudCutout(pointCloudProcessor_.transformPointcloud(downsampledAggregatedPc, selfModel_.getPosition().toTf().inverted()),
                                                                   rtl::BoundingBox3D<float>{rtl::Vector3D<float>{-30.0f, -10.0f, -0.5f},
                                                                                             rtl::Vector3D<float>{ 30.0f,  10.0f, 10.0f}});
            mut downsampledTunel = pointCloudProcessor_.downsamplePointCloud(tunel);
            mut lidarObstacles = lidarObjectDetector_.detectObstacles(downsampledTunel);
            localMap_.setLidarDetections(
                    objectAggregator_.aggregateLidarDetections(localMap_.getLidarDetections(),
                                                               lidarObstacles));


            visualizationHandler_.drawAggregatedPointcloud(downsampledAggregatedPc);
            visualizationHandler_.drawLidarDetection(lidarObstacles);
            visualizationHandler_.drawPointcloudCutout(tunel);
            visualizationHandler_.drawLidarDetection(localMap_.getLidarDetections());
        }

        if (!Center_Lidar_Only) {
            visualizationHandler_.drawRGBImage(imgData);
        }
    }


    void MapBuilder::processIRCameraData(std::shared_ptr<DataModels::CameraIrFrameDataModel> irCameraFrame, std::string& sensorFrame) {

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
        if (!Center_Lidar_Only) {
            visualizationHandler_.drawIRImage(irCameraFrame);
        }
    }


    void MapBuilder::processGnssPoseData(std::shared_ptr<DataModels::GnssPoseDataModel> poseData, std::string& sensorFrame) {

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


    void MapBuilder::processImuDQuatData(std::shared_ptr<DataModels::ImuDquatDataModel> dQuatData, std::string& sensorFrame) {
        selfModel_.onImuDquatData(dQuatData);
    }


    void MapBuilder::processImuGnssData(std::shared_ptr<DataModels::ImuGnssDataModel> poseData, std::string& sensorFrame) {
        imuPoseLogger_.onImuGps(poseData);
        visualizationHandler_.drawImuGpsTrajectory(imuPoseLogger_.getPositionHistory());
    }


    void MapBuilder::processImuImuData(std::shared_ptr<DataModels::ImuImuDataModel> imuData, std::string& sensorFrame) {

        imuProcessor_.setOrientation(imuData->getOrientation());
        mut linAccNoGrav = imuProcessor_.removeGravitaionAcceleration(imuData->getLinearAcc());

        selfModel_.onImuImuData(imuData);
        visualizationHandler_.drawImuData(linAccNoGrav);
    }


    void MapBuilder::processLidarScanData(std::shared_ptr<DataModels::LidarScanDataModel> lidarData, std::string& sensorFrame) {

        let lidarID = lidarData->getLidarIdentifier();

        if (cache_.getLidarScanNo(lidarID) >= 0) {
            aggregateLidar(lidarData);
            approximateLidar(lidarData);
        }

        if (Center_Lidar_Only) {
            if (cache_.getRGBFrameNo(DataLoader::CameraIndentifier::kCameraLeftFront) > 0) {
                visualizationHandler_.drawRGBImage(cache_.getRGBFrame(DataLoader::CameraIndentifier::kCameraLeftFront));
            }
            if (cache_.getIRFrameNo(DataLoader::CameraIndentifier::kCameraIr) > 0) {
                visualizationHandler_.drawIRImage(cache_.getIRFrame(DataLoader::CameraIndentifier::kCameraIr));
            }
        }

        visualizationHandler_.drawLidarData(lidarData);
        visualizationHandler_.drawSelf();
    }

    void MapBuilder::processRadarTiData(std::shared_ptr<DataModels::RadarTiDataModel> data, std::string& sensorFrame) {
        let radarData = std::dynamic_pointer_cast<DataModels::RadarTiDataModel>(data);
        visualizationHandler_.drawRadarTiObjects(radarData->getObjects());
    }


    void MapBuilder::generateDepthMapForRGBFrame(std::shared_ptr<DataModels::CameraFrameDataModel> rgbImg) {

        static size_t cnt = 0;
        if (Depth_Map_For_RGB_Left_Front || Depth_Map_For_RGB_Virtual) {
            let originToImuTf = selfModel_.getPosition().toTf();

            if (Depth_Map_For_RGB_Left_Front) {
                mut points2Dand3Dpair = depthMap_.getPointsInCameraFoV(rgbImg->getCameraIdentifier(),
                                                                       rgbImg->getImage().cols,
                                                                       rgbImg->getImage().rows,
                                                                       originToImuTf,
                                                                       false);
                mut depthImg = simpleImageProcessor_.renderLidarPointsToImg(points2Dand3Dpair->first,
                                                                            points2Dand3Dpair->second,
                                                                            rgbImg->getImage().cols,
                                                                            rgbImg->getImage().rows, 11);

                let depthMap = std::make_shared<DataModels::DepthMapDataModel>(rgbImg->getTimestamp(),
                                                                               depthImg.clone(),
                                                                               DataLoader::CameraIndentifier::kCameraLeftFront,
                                                                               std::vector<std::shared_ptr<DataModels::YoloDetection>>{});
                let lidar_data = pointCloudAggregator_.getAggregatedPointCloudWithTf(originToImuTf.inverted());
                let bird_view = occGrid_.create_bird_view_from_data(lidar_data,
                                                                    Algorithms::OccupancyGrid3D::BirdViewParams(15.0, 15.0, 50.0, 0.0, 0.1));

                let spherical_pointcloud = Algorithms::SphericalPoint<float>::fromPointCloud(*lidar_data);
                let spherical_img = simpleImageProcessor_.renderSphericalPointsToImg(spherical_pointcloud, 15.0, 45.0, 50.0, 0.05, 5);

                imageWriter_.saveImage(depthMap->getImage(), cnt, DataLoader::Folders::kOutputDepthMap + "depth_rgb/", "", "png");
                imageWriter_.saveImage(bird_view, cnt, DataLoader::Folders::kOutputDepthMap + "bird_view/", "", "png");
                imageWriter_.saveImage(spherical_img, cnt, DataLoader::Folders::kOutputDepthMap + "sphere/", "", "png");
            }
            if (Depth_Map_For_RGB_Virtual) {
                mut points2Dand3Dpair = depthMap_.getPointsInCameraFoV(DataLoader::CameraIndentifier::kCameraVirtual,
                                                                       rgbImg->getImage().cols,
                                                                       rgbImg->getImage().rows,
                                                                       originToImuTf,
                                                                       false);
                mut depthImg = simpleImageProcessor_.renderLidarPointsToImg(points2Dand3Dpair->first,
                                                                            points2Dand3Dpair->second,
                                                                            rgbImg->getImage().cols,
                                                                            rgbImg->getImage().rows, 11);
                mut depthImgWithHeight = simpleImageProcessor_.renderLidarPointsToImg(points2Dand3Dpair->first,
                                                                            points2Dand3Dpair->second,
                                                                            rgbImg->getImage().cols,
                                                                            rgbImg->getImage().rows, 11);

                let depthMap = std::make_shared<DataModels::DepthMapDataModel>(rgbImg->getTimestamp(),
                                                                               depthImg.clone(),
                                                                               DataLoader::CameraIndentifier::kCameraVirtual,
                                                                               std::vector<std::shared_ptr<DataModels::YoloDetection>>{});
                let depthMapWithHeight = std::make_shared<DataModels::DepthMapDataModel>(rgbImg->getTimestamp(),
                                                                               depthImgWithHeight.clone(),
                                                                               DataLoader::CameraIndentifier::kCameraVirtual,
                                                                               std::vector<std::shared_ptr<DataModels::YoloDetection>>{});

                imageWriter_.saveImage(depthMap->getImage(), cnt, DataLoader::Folders::kOutputDepthMap + "virtual_cam/", "", "png");
                imageWriter_.saveImage(depthMap->getImage(), cnt, DataLoader::Folders::kOutputDepthMap + "virtual_cam_height/", "", "png");
            }
            cnt++;
        }
    }

    void MapBuilder::generateDepthMapForLastIR(std::shared_ptr<DataModels::CameraFrameDataModel> rgbImg) {

        if (cache_.getIRFrameNo(DataLoader::CameraIndentifier::kCameraIr) < 0) {
            return;
        }

        let irFrame = cache_.getIRFrame(DataLoader::CameraIndentifier::kCameraIr);
        if (Depth_Map_For_IR) {
            let originToImuTf = selfModel_.getPosition().toTf();
            mut points2Dand3Dpair = depthMap_.getPointsInCameraFoV(irFrame->getCameraIdentifier(),
                                                                   irFrame->getImage().cols,
                                                                   irFrame->getImage().rows,
                                                                   originToImuTf,
                                                                   false);
            mut depthImg = simpleImageProcessor_.renderLidarPointsToImg(points2Dand3Dpair->first,
                                                                        points2Dand3Dpair->second,
                                                                        irFrame->getImage().cols,
                                                                        irFrame->getImage().rows, 3);

            let depthMap = std::make_shared<DataModels::DepthMapDataModel>(rgbImg->getTimestamp(),
                                                                           depthImg.clone(),
                                                                           DataLoader::CameraIndentifier::kCameraIr,
                                                                           std::vector<std::shared_ptr<DataModels::YoloDetection>>{});

            cache_.setNewDepthMap(depthMap);
            if (cache_.getIRFrameNo(DataLoader::CameraIndentifier::kCameraIr) >= 0 ||
                cache_.getDepthMapNo(DataLoader::CameraIndentifier::kCameraIr) >= 0) {

                let frameCnt = cache_.getIRFrameNo(DataLoader::CameraIndentifier::kCameraIr);

                imageWriter_.saveImage(cache_.getDepthMap(DataLoader::CameraIndentifier::kCameraIr)->getImage(), frameCnt, DataLoader::Folders::kOutputDepthMap, "_depth", "png");
                let a = cache_.getDepthMap(DataLoader::CameraIndentifier::kCameraIr);
                mut rgb_cutout = simpleImageProcessor_.convertLeftFrontRGBToIrFieldOfView(rgbImg->getImage());
                imageWriter_.saveImage(rgb_cutout, frameCnt, DataLoader::Folders::kOutputDepthMap, "_rgb", "jpeg");
                imageWriter_.saveImage(cache_.getIRFrame(DataLoader::CameraIndentifier::kCameraIr)->getImage(), frameCnt, DataLoader::Folders::kOutputDepthMap, "_ir", "jpeg");
            }
        }
    }

    void MapBuilder::projectRGBDetectionsToIR(std::shared_ptr<DataModels::CameraFrameDataModel> imgData, std::vector<std::shared_ptr<const DataModels::FrustumDetection>> frustums) {

        let sensorFrame = getFrameForData(imgData);
        if (RGB_Detection_To_IR_Projection) {
            if (sensorFrame == LocalMap::Frames::kCameraLeftFront) {

                let irFrameNo = cache_.getIRFrameNo(DataLoader::CameraIndentifier::kCameraIr);
                if (irFrameNo < 0) {
                    return;
                }

                mut imuToCameraTf = context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kCameraIr);
                mut reprojectedYoloDetections = yoloIrReprojector_.onNewDetections(frustums, imuToCameraTf.inverted());

                let lastIrFrame = cache_.getIRFrame(DataLoader::CameraIndentifier::kCameraIr);
                mut imgWidthHeight = std::pair<int, int>{lastIrFrame->getImage().cols, lastIrFrame->getImage().rows};
                yoloIRDetectionWriter_.writeDetections(reprojectedYoloDetections, irFrameNo);
                yoloIRDetectionWriter_.writeDetectionsAsTrainData(reprojectedYoloDetections, irFrameNo, imgWidthHeight.first, imgWidthHeight.second);
                yoloIRDetectionWriter_.writeIRImageAsTrainData(lastIrFrame, irFrameNo);
            }
        }
    }

    void MapBuilder::aggregateLidar(const std::shared_ptr<DataModels::LidarScanDataModel>& lidarData) {

        let lidarID = lidarData->getLidarIdentifier();
        let sensorFrame = getFrameForData(lidarData);

        if (cache_.getLidarScanNo(lidarID) < 0) {
            return;
        }

        mut lidarTF = context_.tfTree_.getTransformationForFrame(sensorFrame);
        let lastLidarTimestamp = (cache_.getLidarScan(lidarID))->getTimestamp();
        mut poseBefore = selfModel_.estimatePositionInTime( lastLidarTimestamp );
        mut poseNow = selfModel_.getPosition();
        mut poseDiff = poseNow-poseBefore;

        if (Short_Term_Lidar_Aggregation) {
            mut downsampledScan = pointCloudProcessor_.downsamplePointCloud(lidarData->getScan());
            mut batches = pointCloudExtrapolator_.splitPointCloudToBatches(downsampledScan, poseBefore, poseDiff, lidarTF);
            pointCloudAggregator_.filterOutBatches(lidarData->getTimestamp());
            pointCloudAggregator_.addPointCloudBatches(batches);
        }

        if (Global_Lidar_Aggregation && Short_Term_Lidar_Aggregation) {
            mut aggregatedPointcloud = pointCloudAggregator_.getAggregatedPointCloud();
            mut downsampledAggregatedPc = pointCloudProcessor_.downsamplePointCloud(aggregatedPointcloud);
            globalPointcloudStorage_.addMorePointsToGlobalStorage(downsampledAggregatedPc);
            visualizationHandler_.drawGlobalPointcloud(globalPointcloudStorage_.getEntirePointcloud());
        }
    }


    void MapBuilder::approximateLidar(const std::shared_ptr<DataModels::LidarScanDataModel>& lidarData) {

        let lidarID = lidarData->getLidarIdentifier();
        let sensorFrame = getFrameForData(lidarData);

        if (cache_.getLidarScanNo(lidarID) >= 0) {
            return;
        }

        mut lidarTF = context_.tfTree_.getTransformationForFrame(sensorFrame);
        let lastLidarTimestamp = (cache_.getLidarScan(lidarID))->getTimestamp();
        mut poseBefore = selfModel_.estimatePositionInTime( lastLidarTimestamp );
        mut poseNow = selfModel_.getPosition();
        mut poseDiff = poseNow-poseBefore;

        if (Lidar_Laser_Approx_And_Seg) {
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

            for (mut& approximation : *leftLaserSegmenter_.getAllApproximations()) { approximations->push_back(approximation); }
            for (mut& approximation : *rightLaserSegmenter_.getAllApproximations()) { approximations->push_back(approximation); }

            for (mut& road : *leftLaserSegmenter_.getAllRoads()) { roadApproximations->push_back(road); }
            for (mut& road : *rightLaserSegmenter_.getAllRoads()) { roadApproximations->push_back(road); }

            visualizationHandler_.drawLidarApproximations(approximations);
            visualizationHandler_.drawLidarApproximationsRoad(roadApproximations);
        }
    }


    std::string MapBuilder::getFrameForData(const std::shared_ptr<DataModels::GenericDataModel>& data) {
        let type = data->getType();
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
                    case DataLoader::CameraIndentifier::kCameraVirtual:
                        return LocalMap::Frames::kCameraVirtual;
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
                    case DataLoader::LidarIdentifier::kCenterLidar:
                        return LocalMap::Frames::kLidarCenter;
                    default:
                        context_.logger_.error("Unable to estimate lidar frame!");
                        return LocalMap::Frames::kErr;
                }
            case DataModels::DataModelTypes::kRadarTiScanDataModelType:
                return LocalMap::Frames::kRadarTi;

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