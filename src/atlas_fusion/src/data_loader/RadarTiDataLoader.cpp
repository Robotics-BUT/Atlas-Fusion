/*
 * Copyright 2021 Brno University of Technology
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

#include "data_loader/RadarTiDataLoader.h"
#include "data_loader/RecordingConstants.h"

namespace AtlasFusion {
    namespace DataLoader {


        bool RadarTiDataLoader::loadData(std::string path) {

            std::string folder;
            switch (radarIdentifier_) {
                case RadarIdentifier::kRadarTi:
                    folder = Folders::kRadarTi;
                    break;
                default:
                    context_.logger_.error("Unexpected Radar data loader identifier.");
                    break;
            }

            auto csvContent = readCsv(path + folder + Files::kRadarTiScan);
            for (const auto &substrings : csvContent) {
                if (substrings.size() >= 2) {

                    size_t timestamp = std::stoull(substrings.at(0));
                    size_t no_of_measurements = std::stoull(substrings.at(1));

                    std::vector<DataModels::RadarTiDataModel::Object> objects;
                    for (size_t i = 0; i < no_of_measurements; i++) {
                        size_t offset = 2 + (4 * i);
                        float x = std::stof(substrings.at(offset + 0));
                        float y = std::stof(substrings.at(offset + 1));
                        float z = std::stof(substrings.at(offset + 2));
                        float vel = std::stof(substrings.at(offset + 3));
                        objects.emplace_back(DataModels::RadarTiDataModel::Object{{x,y,z}, vel});
                    }
                    data_.push_back(std::make_shared<DataModels::RadarTiDataModel>(timestamp, objects));

                } else {
                    context_.logger_.warning("Radar frame to short!");
                }
            }
            dataIt_ = data_.begin();
            releaseIt_ = dataIt_;
            return true;
        }

        timestamp_type RadarTiDataLoader::getLowestTimestamp() {

            if(!isOnEnd()) {
                return (*dataIt_)->getTimestamp();
            }
            return std::numeric_limits<uint64_t>::max();
        }

        std::shared_ptr<DataModels::GenericDataModel> RadarTiDataLoader::getNextData() {
            if (!isOnEnd()) {
                auto output = *dataIt_;
                dataIt_ = std::next(dataIt_,1);
                return output;
            }
            return std::make_shared<DataModels::ErrorDataModel>();
        }

        std::string RadarTiDataLoader::toString() {
            std::stringstream ss;
            ss << "[Radar Ti Data Loader] : size = " << data_.size();
            return ss.str();
        }

        uint64_t RadarTiDataLoader::getDataSize() {
            return data_.size();
        }

        bool RadarTiDataLoader::isOnEnd() {

            return dataIt_ >= data_.end();
        }

        void RadarTiDataLoader::setPose(timestamp_type timestamp) {
            for (dataIt_ = data_.begin(); dataIt_< data_.end() ; dataIt_ = std::next(dataIt_,1)) {
                if((*dataIt_)->getTimestamp() >= timestamp) {
                    break;
                }
            }
        }

        void RadarTiDataLoader::releaseOldData(timestamp_type keepHistory) {
            auto currentTime = (*dataIt_)->getTimestamp();
            while(releaseIt_ < data_.end()) {
                auto dataTimestamp = (*releaseIt_)->getTimestamp();
                if(dataTimestamp + keepHistory < currentTime) {
                    (*releaseIt_) = std::make_shared<DataModels::RadarTiDataModel>(0);
                    releaseIt_++;
                } else {
                    break;
                }
            }
        }

        void RadarTiDataLoader::clear() {
            data_.clear();
            dataIt_ = data_.begin();
        }

    }
}