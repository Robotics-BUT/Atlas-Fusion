#include "data_loader/DataLoader.h"
#include "data_loader/RecordingConstants.h"

#include <iostream>
#include <sstream>
#include <experimental/filesystem>
#include <fstream>
#include <iomanip>
#include <local_map/Frames.h>

namespace AutoDrive {
    namespace DataLoader {

        bool DataLoader::loadData(std::string path) {

            if(!std::experimental::filesystem::is_directory(path)) {
                context_.logger_.error("Unable to open recording on path: " + path);
                return false;
            }

            auto isConsistent = checkRecordConsistency(path);
            if(!isConsistent) {
                context_.logger_.error("Recording is unconsistent");
                return false;
            }

            clear();
            auto isOk = loadRecord(path);

            return isOk;
        }


        timestamp_type DataLoader::getLowestTimestamp() {
            uint64_t minTimestamp = std::numeric_limits<uint64_t>::max();

            for(const auto& dataLoader : dataLoaders_) {
                if (dataLoader->getLowestTimestamp() < minTimestamp){
                    minTimestamp = dataLoader->getLowestTimestamp();
                }
            }
            return minTimestamp;
        }



        std::string DataLoader::toString() {
            return "";
        }


        uint64_t DataLoader::getDataSize() {
            uint32_t sum = 0;
            for(const auto& dataLoader : dataLoaders_) {
                sum += dataLoader->getDataSize();
            }
            return sum;
        }


        bool DataLoader::isOnEnd() {
            for(const auto& dataLoader : dataLoaders_) {
                if(!dataLoader->isOnEnd()) {
                    return false;
                }
            }
            return true;
        }

        void DataLoader::setPose(timestamp_type pose) {
            for(const auto& dataLoader : dataLoaders_) {
                dataLoader->setPose(pose);
            }
        }

        std::shared_ptr<DataModels::GenericDataModel> DataLoader::getNextData() {

            std::shared_ptr<AbstractDataLoader> it = nullptr;
            uint64_t minTimestamp = std::numeric_limits<uint64_t>::max();

            for(const auto& dataLoader : dataLoaders_) {
                if (dataLoader->getLowestTimestamp() < minTimestamp){
                    minTimestamp = dataLoader->getLowestTimestamp();
                    it = dataLoader;
                }
            }

            if (it != nullptr && minTimestamp < std::numeric_limits<uint64_t>::max()) {
                auto ret = it->getNextData();
                it->releaseOldData(keepHistoryLength_);
                return ret;
            }
            return std::make_shared<DataModels::ErrorDataModel>();
        }


        void DataLoader::clear() {
            for(const auto& dataLodar : dataLoaders_) {
                dataLodar->clear();
            }
        }


        void DataLoader::releaseOldData(timestamp_type keepHistory) {
            for(const auto& dataLodar : dataLoaders_) {
                dataLodar->releaseOldData(keepHistory);
            }
        }


        bool DataLoader::checkRecordConsistency(std::string& path) {

            for(const auto& folder : Folders::mandatoryFolders) {
                const auto directory = path + folder;
                if(!std::experimental::filesystem::is_directory(directory)) {
                    context_.logger_.error("Unable to find mandatory folder: " + directory);
                    return false;
                }
            }

            for(const auto& file : Files::mandatoryFiles) {
                const auto f = path + file;
                if(!std::experimental::filesystem::is_regular_file(f)) {
                    context_.logger_.error("Unable to find mandatory file: " + f);
                    return false;
                }
            }

            return true;
        }

        bool DataLoader::loadRecord(std::string& path) {

            context_.logger_.info("Loading data ...");

            for(const auto& dataLoader : dataLoaders_) {
                auto result = dataLoader->loadData(path);
                context_.logger_.info(dataLoader->toString());
                if (!result) {
                    context_.logger_.error("Error when reading data from filesystem");
                    return false;
                }
            }

            context_.logger_.info("Data Loading done");
            return true;
        }


        CameraIndentifier DataLoader::getCameraIDfromFrame(const std::string& frame) {
            if (frame == LocalMap::Frames::kCameraLeftFront) {
                return CameraIndentifier::kCameraLeftFront;
            } else if (frame == LocalMap::Frames::kCameraLeftSide) {
                return CameraIndentifier::kCameraLeftSide;
            } else if (frame == LocalMap::Frames::kCameraRightFront) {
                return CameraIndentifier::kCameraRightFront;
            } else if (frame == LocalMap::Frames::kCameraRightSide) {
                return CameraIndentifier::kCameraRightSide;
            } else if (frame == LocalMap::Frames::kCameraIr) {
                return CameraIndentifier::kCameraIr;
            } else {
                context_.logger_.warning("Unexpected camera frame in DataLoader::getCameraIDfromFrame()!");
                return CameraIndentifier::kErr;
            };
        }

        std::shared_ptr<DataModels::GenericDataModel> DataLoader::getCameraCalibDataForCameraID(CameraIndentifier id) {
            for (const auto& cameraDataLoader : cameraDataLoaders_) {
                if(cameraDataLoader->getCameraIdentifier() == id) {
                    return cameraDataLoader->getCameraCalibParams();
                }
            }
            return std::make_shared<DataModels::ErrorDataModel>();
        }
    }
}