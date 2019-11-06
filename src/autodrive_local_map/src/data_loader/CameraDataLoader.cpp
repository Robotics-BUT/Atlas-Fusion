#include "data_loader/CameraDataLoader.h"

#include <opencv2/opencv.hpp>
#include <sstream>

#include "data_loader/RecordingConstants.h"

namespace AutoDrive {
    namespace DataLoader {

        bool CameraDataLoader::loadData(std::string path) {

            data_.clear();
            std::string folder;
            switch (cameraIdentifier_){
                case CameraIndentifier::kCameraLeftFront:
                    folder = Folders::kCameraLeftFrontFolder; break;
                case CameraIndentifier::kCameraLeftSide:
                    folder = Folders::kCameraLeftSideFolder; break;
                case CameraIndentifier::kCameraRightFront:
                    folder = Folders::kCameraRightFrontFolder; break;
                case CameraIndentifier::kCameraRightSide:
                    folder = Folders::kCameraRightSideFolder; break;
                case CameraIndentifier::kCameraIr:
                    folder = Folders::kCameraIr; break;
            }

            auto csvContent = readCsv(path + folder + Files::kTimestampFile);
            for(const auto& substrings : csvContent) {
                switch(cameraIdentifier_) {
                    case CameraIndentifier::kCameraLeftFront:
                    case CameraIndentifier::kCameraLeftSide:
                    case CameraIndentifier::kCameraRightFront:
                    case CameraIndentifier::kCameraRightSide:
                        if (substrings.size() == 3) {
                            std::stringstream ss;
                            data_.push_back(
                                    std::make_shared<CameraFrame>(std::stoull(substrings[0]), std::stol(substrings[1]), std::stoull(substrings[2]), 0.0, 0.0));
                        } else {
                            std::cerr << "Unexpected lenght of camera rgb timestamp row: " << std::endl;
                        }
                        break;

                    case CameraIndentifier::kCameraIr:
                        if (substrings.size() == 4) {
                            std::stringstream ss;
                            data_.push_back(
                                    std::make_shared<CameraFrame>(std::stoull(substrings[0]), std::stol(substrings[1]), 0, std::stod(substrings[2]), std::stod(substrings[3])));
                        } else {
                            std::cerr << "Unexpected lenght of camera ir timestamp row: " << std::endl;
                        }
                        break;
                }
            }

            auto videoPath = path + folder + Files::kVideoFile;
            video_.open(videoPath);
            if(!video_.isOpened()) {
                std::cerr << "Unable to open video file: " << videoPath << std::endl;
                clear();
                return false;
            }

            if (data_.size() != video_.get(cv::CAP_PROP_FRAME_COUNT)) {
                std::cerr << "Number of timestamps is no equal to number of frames: " << data_.size() << " != " << video_.get(cv::CAP_PROP_FRAME_COUNT) << std::endl;
                clear();
                return false;
            }

            if(cameraIdentifier_ == CameraIndentifier::kCameraLeftFront ||
               cameraIdentifier_ == CameraIndentifier::kCameraLeftSide ||
               cameraIdentifier_ == CameraIndentifier::kCameraRightFront ||
               cameraIdentifier_ == CameraIndentifier::kCameraRightSide) {
                    std::string yoloFile = folder.replace(folder.find('/'),std::string("/").length(),".txt");
                    loadYoloDetections(path + Folders::kYoloFolder + yoloFile);
            }

            dataIt_ = data_.begin();
            return true;
        }

        timestamp_type CameraDataLoader::getLowestTimestamp() {
            if(!isOnEnd()) {
                return (*dataIt_)->timestamp_;
            }
            return std::numeric_limits<uint64_t>::max();
        }

        std::shared_ptr<GenericDataModel> CameraDataLoader::getNextData() {
            if (!isOnEnd()) {
                cv::Mat frame;
                video_ >> frame;

                std::shared_ptr<GenericDataModel> cameraFrame;
                switch(cameraIdentifier_) {
                    case CameraIndentifier::kCameraLeftFront:
                    case CameraIndentifier::kCameraLeftSide:
                    case CameraIndentifier::kCameraRightFront:
                    case CameraIndentifier::kCameraRightSide:
                        cameraFrame = std::make_shared<CameraFrameDataModel>((*dataIt_)->timestamp_, frame, (*dataIt_)->innerTimestamp_, (*dataIt_)->detections_);

                        break;

                    case CameraIndentifier::kCameraIr:
                        cameraFrame = std::make_shared<CameraIrFrameDataModel>((*dataIt_)->timestamp_, frame, (*dataIt_)->tempMin_, (*dataIt_)->tempMax_);
                        break;
                }

                dataIt_ = std::next(dataIt_, 1);
                return cameraFrame;
            }
            return std::make_shared<ErrorDataModel>();
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

        void CameraDataLoader::rewind() {
            dataIt_ = data_.begin();
        }

        void CameraDataLoader::setPose(timestamp_type timestamp) {
            for (dataIt_ = data_.begin(); dataIt_< data_.end() ; dataIt_ = std::next(dataIt_,1)) {
                if((*dataIt_)->getTimestamp() >= timestamp) {
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
                        std::cout << "Unexpected lenght of yolo detection in csv" << std::endl;
                        continue;
                    }

                    size_t frame_id = std::stoul(detection[0]);
                    if(frame_id < data_.size()) {
                        data_.at(frame_id)->detections_.push_back(std::make_shared<YoloDetection>(
                                std::stoul(detection[1]), std::stoul(detection[2]),
                                std::stoul(detection[3]), std::stoul(detection[4]),
                                std::stof(detection[5]), std::stof(detection[6]),
                                static_cast<YoloDetectionClasses>(std::stoi(detection[7]))));
                    }
                }
            }
        }
    }
}