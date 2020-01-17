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

        //dataLoader_.setPose(1564663008747159969);

        std::vector<DataLoader::CameraIndentifier> cameraIDs = {
                DataLoader::CameraIndentifier::kCameraLeftFront,
                DataLoader::CameraIndentifier::kCameraLeftSide,
                DataLoader::CameraIndentifier::kCameraRightFront,
                DataLoader::CameraIndentifier::kCameraRightSide,
                DataLoader::CameraIndentifier::kCameraIr,
        };

        for(const auto& cam : cameraIDs) {
            auto params = dataLoader_.getCameraCalibDataForCameraID(cam);
            auto tf = getCameraTf(cam);

            if(params->getType() == DataModels::DataModelTypes::kCameraCalibrationParamsDataModelType) {
                auto paramModel = std::dynamic_pointer_cast<DataModels::CameraCalibrationParamsDataModel>(params);
                visualizationHandler_.setCameraCalibParamsForCameraId(paramModel, cam);

                auto intrinsicParams = paramModel->getMatIntrinsicParams();
                auto distortionParams =  paramModel->getMatDistortionParams();

                auto projector = std::make_shared<Algorithms::Projector>(intrinsicParams, distortionParams, tf);
                depthMap_.addProjector(projector, cam);
                detectionProcessor_.addProjector(projector, getCameraFrame(cam));
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

            if(dataCounter++ > 1000) {
                return;
            }

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


            /* ... data processing ... */

            if(dataType == DataModels::DataModelTypes::kCameraDataModelType) {

                static int cnt = 0;

                auto cameraFrame = std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(data);
//                if(cameraFrame->getCameraIdentifier() == DataLoader::CameraIndentifier::kCameraRightSide) {

                    auto batches = pointCloudAggregator_.getAllBatches();
                    depthMap_.updatePointcloudData(batches);

                    auto detections3D = depthMap_.onNewCameraData(cameraFrame, selfModel_.getPosition());
                    auto frustums = detectionProcessor_.onNew3DYoloDetections(detections3D, getCameraFrame(
                            cameraFrame->getCameraIdentifier()));

                    localMap_.onNewFrustumDetections(frustums, getCameraFrame(cameraFrame->getCameraIdentifier()));
                    visualizationHandler_.drawFrustumDetections(localMap_.getAllFrustumDetections());
//                }

                if(cnt++ >= 3) {
                    auto aggregatedPointcloud = pointCloudAggregator_.getAggregatedPointCloud();
                    auto downsampledAggregatedPc = pointCloudProcessor_.downsamplePointCloud(aggregatedPointcloud);
                    visualizationHandler_.drawAggregatedPointcloud(downsampledAggregatedPc);
                    cnt = 0;
                }

                visualizationHandler_.drawRGBImage(cameraFrame);

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
                auto lidarFrame = getLidarFrame(lidarID);

                if (lidarDataHistory_.count(lidarFrame) > 0) {

                    auto lidarTF = context_.tfTree_.getTransformationForFrame(getLidarFrame(lidarID));
                    auto lastLidarTimestamp = (lidarDataHistory_[lidarFrame])->getTimestamp();
                    auto poseBefore = selfModel_.estimatePositionInTime( lastLidarTimestamp );
                    auto poseNow = selfModel_.getPosition();

                    auto downsampledScan = pointCloudProcessor_.downsamplePointCloud(lidarData->getScan());

                    auto batches = pointCloudExtrapolator_.splitPointCloudToBatches(downsampledScan, poseNow, poseNow-poseBefore, lidarTF);
                    pointCloudAggregator_.filterOutBatches(lidarData->getTimestamp());
                    pointCloudAggregator_.addPointCloudBatches(batches);

                }
                lidarDataHistory_[lidarFrame] = lidarData;

//                depthMap_.onNewLidarData(lidarData);
                visualizationHandler_.drawLidarData(lidarData);

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


    rtl::Transformation3D<double> MapBuilder::getCameraTf(const DataLoader::CameraIndentifier& id) {

        switch (id){
            case DataLoader::CameraIndentifier::kCameraLeftFront:
                return context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kCameraLeftFront);
            case DataLoader::CameraIndentifier::kCameraLeftSide:
                return context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kCameraLeftSide);
            case DataLoader::CameraIndentifier::kCameraRightFront:
                return context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kCameraRightFront);
            case DataLoader::CameraIndentifier::kCameraRightSide:
                return context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kCameraRightSide);
            case DataLoader::CameraIndentifier::kCameraIr:
                return context_.tfTree_.getTransformationForFrame(LocalMap::Frames::kCameraIr);
            default:
                context_.logger_.error("Unable to find transformation for camera!");
                return rtl::Transformation3D<double>();
        }
    }

    std::string MapBuilder::getCameraFrame(const DataLoader::CameraIndentifier& id) {

        switch (id){
            case DataLoader::CameraIndentifier::kCameraLeftFront:
                return LocalMap::Frames::kCameraLeftFront;
            case DataLoader::CameraIndentifier::kCameraLeftSide:
                return LocalMap::Frames::kCameraLeftSide;
            case DataLoader::CameraIndentifier::kCameraRightFront:
                return LocalMap::Frames::kCameraRightFront;
            case DataLoader::CameraIndentifier::kCameraRightSide:
                return LocalMap::Frames::kCameraRightSide;
            case DataLoader::CameraIndentifier::kCameraIr:
                return LocalMap::Frames::kCameraIr;
            default:
                context_.logger_.error("Unable to find frame for camera!");
                return "";
        }
    }


    std::string MapBuilder::getLidarFrame(const DataLoader::LidarIdentifier & id) {
        switch (id){
            case DataLoader::LidarIdentifier::kLeftLidar:
                return LocalMap::Frames::kLidarLeft;
            case DataLoader::LidarIdentifier::kRightLidar:
                return LocalMap::Frames::kLidarRight;
            default:
                context_.logger_.error("Unable to find frame for lidar!");
                return "";
        }
    }
}