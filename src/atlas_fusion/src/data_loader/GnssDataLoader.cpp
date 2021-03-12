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

#include "data_loader/GnssDataLoader.h"

#include "data_loader/RecordingConstants.h"

namespace AtlasFusion {
    namespace DataLoader {

        bool GnssDataLoader::loadData(std::string path) {

            switch(dataLoaderIdentifier_) {
                case GnssLoaderIdentifier::kPose:
                    data_ = loadGnssPoseData(path); break;
                case GnssLoaderIdentifier::kTime:
                    data_ = loadGnssTimeData(path); break;
            }
            dataIt_ = data_.begin();
            releaseIt_ = dataIt_;
            return true;
        }

        timestamp_type GnssDataLoader::getLowestTimestamp() {
            if(!isOnEnd()) {
                return (*dataIt_)->getTimestamp();
            }
            return std::numeric_limits<uint64_t>::max();
        }

        std::shared_ptr<DataModels::GenericDataModel> GnssDataLoader::getNextData() {
            if (!isOnEnd()) {
                auto output = *dataIt_;
                dataIt_ = std::next(dataIt_,1);
                return output;
            }
            return std::make_shared<DataModels::ErrorDataModel>();
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

        void GnssDataLoader::setPose(timestamp_type timestamp) {
            for (dataIt_ = data_.begin(); dataIt_< data_.end() ; dataIt_ = std::next(dataIt_,1)) {
                if((*dataIt_)->getTimestamp() >= timestamp) {
                    break;
                }
            }
        }

        void GnssDataLoader::releaseOldData(timestamp_type keepHistory) {
            auto currentTime = (*dataIt_)->getTimestamp();
            while(releaseIt_ < data_.end()) {
                auto dataTimestamp = (*releaseIt_)->getTimestamp();
                if(dataTimestamp + keepHistory < currentTime) {
                    (*releaseIt_) = std::make_shared<DataModels::GenericDataModel>(0);
                    releaseIt_++;
                } else {
                    break;
                }
            }
        }

        void GnssDataLoader::clear() {
            data_.clear();
            dataIt_ = data_.begin();
        }

        std::vector<std::shared_ptr<DataModels::GenericDataModel>> GnssDataLoader::loadGnssTimeData(std::string& path) {
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kGnssFolder + Files::kTimeFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 8) {
                    output.push_back( std::make_shared<DataModels::GnssTimeDataModel>(
                            std::stoll(substrings[0]),std::stoll(substrings[1]),
                            std::stoll(substrings[2]),std::stoll(substrings[3]),
                            std::stoll(substrings[4]),std::stoll(substrings[5]),
                            std::stoll(substrings[6]),std::stoll(substrings[7])));
                } else {
                    context_.logger_.error("Unexpected lenght of gnss time data");
                }
            }
            return output;
        }


        std::vector<std::shared_ptr<DataModels::GenericDataModel>> GnssDataLoader::loadGnssPoseData(std::string& path) {
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kGnssFolder + Files::kPoseFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 5) {
                    output.push_back( std::make_shared<DataModels::GnssPoseDataModel>(
                            std::stoll(substrings[0]),std::stod(substrings[1]),
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