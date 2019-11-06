#include "data_loader/GnssDataLoader.h"

#include "data_loader/RecordingConstants.h"

namespace AutoDrive {
    namespace DataLoader {

        bool GnssDataLoader::loadData(std::string path) {

            switch(dataLoaderIdentifier_) {
                case GnssLoaderIdentifier::kPose:
                    data_ = loadGnssPoseData(path); break;
                case GnssLoaderIdentifier::kTime:
                    data_ = loadGnssTimeData(path); break;
            }
            dataIt_ = data_.begin();
            return true;
        }

        timestamp_type GnssDataLoader::getLowestTimestamp() {
            if(!isOnEnd()) {
                return (*dataIt_)->getTimestamp();
            }
            return std::numeric_limits<uint64_t>::max();
        }

        std::shared_ptr<GenericDataModel> GnssDataLoader::getNextData() {
            if (!isOnEnd()) {
                auto output = *dataIt_;
                dataIt_ = std::next(dataIt_,1);
                return output;
            }
            return std::make_shared<ErrorDataModel>();
        }

        std::string GnssDataLoader::toString() {
            std::stringstream ss;
            std::string type;

            switch(dataLoaderIdentifier_) {
                case GnssLoaderIdentifier::kPose:
                    type = "Pose"; break;
                case GnssLoaderIdentifier::kTime:
                    type = "Time"; break;
            }

            ss << "[Gnss " << type << " Data Loader] : " << data_.size();
            return ss.str();
        }

        uint64_t GnssDataLoader::getDataSize() {
            return data_.size();
        }

        bool GnssDataLoader::isOnEnd() {

            return dataIt_ >= data_.end();
        }

        void GnssDataLoader::rewind() {
            dataIt_ = data_.begin();
        }

        void GnssDataLoader::setPose(timestamp_type timestamp) {
            for (dataIt_ = data_.begin(); dataIt_< data_.end() ; dataIt_ = std::next(dataIt_,1)) {
                if((*dataIt_)->getTimestamp() >= timestamp) {
                    break;
                }
            }
        }

        void GnssDataLoader::clear() {
            data_.clear();
            dataIt_ = data_.begin();
        }

        std::vector<std::shared_ptr<GenericDataModel>> GnssDataLoader::loadGnssTimeData(std::string& path) {
            std::vector<std::shared_ptr<GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kGnssFolder + Files::kTimeFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 8) {
                    output.push_back( std::make_shared<GnssTimeDataModel>(std::stoll(substrings[0]),std::stoll(substrings[1]),
                                                                          std::stoll(substrings[2]),std::stoll(substrings[3]),
                                                                          std::stoll(substrings[4]),std::stoll(substrings[5]),
                                                                          std::stoll(substrings[6]),std::stoll(substrings[7])));
                } else {
                    std::cerr << "Unexpected lenght of gnss time data: " << std::endl;
                }
            }
            return output;
        }


        std::vector<std::shared_ptr<GenericDataModel>> GnssDataLoader::loadGnssPoseData(std::string& path) {
            std::vector<std::shared_ptr<GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kGnssFolder + Files::kPoseFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 5) {
                    output.push_back( std::make_shared<GnssPoseDataModel>(std::stoll(substrings[0]),std::stod(substrings[1]),
                                                                          std::stod(substrings[2]), std::stod(substrings[3]),
                                                                          std::stod(substrings[4])));
                } else {
                    std::cerr << "Unexpected lenght of gnss pose data: " << std::endl;
                }
            }
            return output;
        }

    }
}