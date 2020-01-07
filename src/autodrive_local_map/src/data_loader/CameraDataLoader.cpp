#include "data_loader/CameraDataLoader.h"

#include <opencv2/opencv.hpp>
#include <sstream>

#include "data_loader/RecordingConstants.h"
#include "ConfigService.h"

namespace AutoDrive::DataLoader {

        bool CameraDataLoader::loadData(std::string path) {

            data_.clear();
            std::string folder;
            switch (cameraIdentifier_){
                case DataLoader::CameraIndentifier::kCameraLeftFront:
                    folder = Folders::kCameraLeftFrontFolder; break;
                case DataLoader::CameraIndentifier::kCameraLeftSide:
                    folder = Folders::kCameraLeftSideFolder; break;
                case DataLoader::CameraIndentifier::kCameraRightFront:
                    folder = Folders::kCameraRightFrontFolder; break;
                case DataLoader::CameraIndentifier::kCameraRightSide:
                    folder = Folders::kCameraRightSideFolder; break;
                case DataLoader::CameraIndentifier::kCameraIr:
                    folder = Folders::kCameraIr; break;
            }

            auto csvContent = readCsv(path + folder + Files::kTimestampFile);
            for(const auto& substrings : csvContent) {
                switch(cameraIdentifier_) {
                    case DataLoader::CameraIndentifier::kCameraLeftFront:
                    case DataLoader::CameraIndentifier::kCameraLeftSide:
                    case DataLoader::CameraIndentifier::kCameraRightFront:
                    case DataLoader::CameraIndentifier::kCameraRightSide:
                        if (substrings.size() == 3) {
                            std::stringstream ss;
                            data_.push_back(
                                    std::make_shared<CameraFrame>(std::stoull(substrings[0]), std::stol(substrings[1]), std::stoull(substrings[2]), 0.0, 0.0));
                        } else {
                            context_.logger_.error("Unexpected lenght of camera rgb timestamp row: ");
                        }
                        break;

                    case DataLoader::CameraIndentifier::kCameraIr:
                        if (substrings.size() == 4) {
                            std::stringstream ss;
                            data_.push_back(
                                    std::make_shared<CameraFrame>(std::stoull(substrings[0]), std::stol(substrings[1]), 0, std::stod(substrings[2]), std::stod(substrings[3])));
                        } else {
                            context_.logger_.error("Unexpected lenght of camera ir timestamp row: ");
                        }
                        break;
                }
            }

            auto videoPath = path + folder + Files::kVideoFile;
            video_.open(videoPath);
            if(!video_.isOpened()) {
                context_.logger_.error("Unable to open video file: " + videoPath);
                clear();
                return false;
            }

            if (data_.size() != video_.get(cv::CAP_PROP_FRAME_COUNT)) {
                std::stringstream ss;
                ss << "Number of timestamps is no equal to number of frames: " << data_.size() << " != " << video_.get(cv::CAP_PROP_FRAME_COUNT);
                context_.logger_.error(ss.str());
                clear();
                return false;
            }

            if(cameraIdentifier_ == DataLoader::CameraIndentifier::kCameraLeftFront ||
               cameraIdentifier_ == DataLoader::CameraIndentifier::kCameraLeftSide ||
               cameraIdentifier_ == DataLoader::CameraIndentifier::kCameraRightFront ||
               cameraIdentifier_ == DataLoader::CameraIndentifier::kCameraRightSide) {
                    std::string yoloFile = folder.replace(folder.find('/'),std::string("/").length(),".txt");
                    loadYoloDetections(path + Folders::kYoloFolder + yoloFile);
            }

            dataIt_ = data_.begin();
            releaseIt_ = dataIt_;
            return true;
        }

        timestamp_type CameraDataLoader::getLowestTimestamp() {
            if(!isOnEnd()) {
                return (*dataIt_)->timestamp_;
            }
            return std::numeric_limits<uint64_t>::max();
        }

        std::shared_ptr<DataModels::GenericDataModel> CameraDataLoader::getNextData() {
            if (!isOnEnd()) {
                cv::Mat frame;
                video_ >> frame;
                cv::Mat undistortFrame(frame.rows, frame.cols, frame.type());

//                auto calibParams = getCameraCalibParams();
//                auto intrinsic = calibParams->getMatIntrinsicParams();
//                auto distortion = calibParams->getMatDistortionParams();

//                cv::undistort(frame, undistortFrame, intrinsic, distortion);
                std::shared_ptr<DataModels::GenericDataModel> cameraFrame;
                switch(cameraIdentifier_) {
                    case DataLoader::CameraIndentifier::kCameraLeftFront:
                    case DataLoader::CameraIndentifier::kCameraLeftSide:
                    case DataLoader::CameraIndentifier::kCameraRightFront:
                    case DataLoader::CameraIndentifier::kCameraRightSide:
                        cameraFrame = std::make_shared<DataModels::CameraFrameDataModel>((*dataIt_)->timestamp_, frame, (*dataIt_)->innerTimestamp_, cameraIdentifier_, (*dataIt_)->detections_);

                        break;

                    case DataLoader::CameraIndentifier::kCameraIr:
                        cameraFrame = std::make_shared<DataModels::CameraIrFrameDataModel>((*dataIt_)->timestamp_, frame, (*dataIt_)->tempMin_, (*dataIt_)->tempMax_);
                        break;
                }

                dataIt_ = std::next(dataIt_, 1);
                return cameraFrame;
            }
            return std::make_shared<DataModels::ErrorDataModel>();
        }


        std::string CameraDataLoader::toString() {
            std::stringstream ss;
            ss << "[Camera Data Loader] : size = " << data_.size() << " video frames = " << video_.get(cv::CAP_PROP_FRAME_COUNT);
            return ss.str();
        }

        uint64_t CameraDataLoader::getDataSize() {
            return data_.size();
        }

        bool CameraDataLoader::isOnEnd() {
            return dataIt_ >= data_.end();
        }

        void CameraDataLoader::setPose(timestamp_type timestamp) {
            cv::Mat frame;
            for (dataIt_ = data_.begin(); dataIt_< data_.end() ; dataIt_ = std::next(dataIt_,1)) {
                video_ >> frame;
                if((*dataIt_)->getTimestamp() >= timestamp) {
                    break;
                }
            }
        }


        void CameraDataLoader::releaseOldData(timestamp_type keepHistory) {
            auto currentTime = (*dataIt_)->getTimestamp();
            while(releaseIt_ < data_.end()) {
                auto dataTimestamp = (*releaseIt_)->getTimestamp();
                if(dataTimestamp + keepHistory < currentTime) {
                    (*releaseIt_) = std::make_shared<CameraFrame>(0, 0, 0, 0, 0);
                    releaseIt_++;
                } else {
                    break;
                }
            }
        }


        void CameraDataLoader::clear() {
            video_.release();
            data_.clear();
            dataIt_ = data_.begin();
        }



        void CameraDataLoader::loadYoloDetections(const std::string& path) const {
            auto csvContent = readCsv(std::string(path));

            if(!csvContent.empty()) {
                for (const auto& detection : csvContent) {
                    if(detection.size() != 8) {
                        context_.logger_.warning("Unexpected lenght of yolo detection in csv");
                        continue;
                    }

                    size_t frame_id = std::stoul(detection[0]);
                    if(frame_id < data_.size()) {
                        data_.at(frame_id)->detections_.push_back(std::make_shared<DataModels::YoloDetection>(
                                std::stoul(detection[1]), std::stoul(detection[2]),
                                std::stoul(detection[3]), std::stoul(detection[4]),
                                std::stof(detection[5]), std::stof(detection[6]),
                                static_cast<DataModels::YoloDetectionClass>(std::stoi(detection[7]))));
                    }
                }
            }
        }


        std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> CameraDataLoader::getCameraCalibParams() {
            auto configService = ConfigService(std::string(cameraCalibFilePath_));
            auto width = configService.getDoubleValue({"width"});
            auto height = configService.getDoubleValue({"height"});
            auto intrinsic = configService.getMatValue<double>({"intrinsic"});
            auto dist = configService.getArrayValue<double>({"dist"});
            return std::make_shared<DataModels::CameraCalibrationParamsDataModel>(width, height, intrinsic, dist);
        }
}