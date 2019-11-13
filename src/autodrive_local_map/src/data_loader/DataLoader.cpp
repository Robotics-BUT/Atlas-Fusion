#include "data_loader/DataLoader.h"
#include "data_loader/RecordingConstants.h"

#include <iostream>
#include <sstream>
#include <experimental/filesystem>
#include <fstream>
#include <iomanip>

namespace AutoDrive {
    namespace DataLoader {

        bool DataLoader::loadData(std::string path) {

            if(!std::experimental::filesystem::is_directory(path)) {
                std::cerr << "Unable to open recording on path: " << path << std::endl;
                return false;
            }

            auto isConsistent = checkRecordConsistency(path);
            if(!isConsistent) {
                std::cerr << "Recording is unconsistent" << std::endl;
                return false;
            }

            clear();
            auto isOk = loadRecord(path);

            return isOk;
        }


        timestamp_type DataLoader::getLowestTimestamp() {
            return 0;
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

        std::shared_ptr<GenericDataModel> DataLoader::getNextData() {

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
            return std::make_shared<ErrorDataModel>();
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
                    std::cerr << "Unable to find mandatory folder: " << directory << std::endl;
                    return false;
                }
            }

            for(const auto& file : Files::mandatoryFiles) {
                const auto f = path + file;
                if(!std::experimental::filesystem::is_regular_file(f)) {
                    std::cerr << "Unable to find mandatory file: " << f << std::endl;
                    return false;
                }
            }

            return true;
        }

        bool DataLoader::loadRecord(std::string& path) {

            std::cout << "Loading data ..." << std::endl;

            for(const auto& dataLoader : dataLoaders_) {
                auto result = dataLoader->loadData(path);
                std::cout << dataLoader->toString() << std::endl;
                if (!result) {
                    std::cerr << "Error when reading data from filesystem" << std::endl;
                    return false;
                }
            }

            std::cout << "Data Loading done" << std::endl;
            return true;
        }
    }
}