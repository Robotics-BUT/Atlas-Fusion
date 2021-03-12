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

#include "data_loader/LidarDataLoader.h"
#include "data_loader/RecordingConstants.h"

namespace AtlasFusion {
    namespace DataLoader {


        bool LidarDataLoader::loadData(std::string path) {

            std::string folder;
            switch (lidarIdentifier_) {
                case LidarIdentifier::kLeftLidar:
                    folder = Folders::kLidarLeftFolder;
                    break;
                case LidarIdentifier::kRightLidar:
                    folder = Folders::kLidarRightFolder;
                    break;
                case LidarIdentifier::kCenterLidar:
                    folder = Folders::kLidarCenterFolder;
                    break;
                default:
                    context_.logger_.error("Unexpected LiDAR data loader identifier.");
                    break;
            }

            auto csvContent = readCsv(path + folder + Files::kTimestampFile);
            for (const auto &substrings : csvContent) {
                size_t timestamp = 0;
                size_t scan_no = 0;
                size_t lidar_timestamp = 0;

                if (substrings.size() == 2) {
                    timestamp = std::stoll(substrings[0]);
                    scan_no = std::stoll(substrings[1]);
                }
                else if (substrings.size() == 3) {
                    timestamp = std::stoll(substrings[0]);
                    scan_no = std::stoll(substrings[1]);
                    lidar_timestamp = std::stoll(substrings[2]);
                }
                else {
                    context_.logger_.error("Unexpected lenght of lidar scan data: ");
                }

                std::stringstream ss;
                ss << path + folder + Files::kScanFile << std::setw(6) << std::setfill('0')
                   << scan_no << Files::kPcdExt;
                data_.push_back(std::make_shared<DataModels::LidarScanDataModel>(timestamp,
                                                                                 lidarIdentifier_,
                                                                                 ss.str(),
                                                                                 lidar_timestamp));
            }
            dataIt_ = data_.begin();
            releaseIt_ = dataIt_;
            return true;
        }

        timestamp_type LidarDataLoader::getLowestTimestamp() {

            if(!isOnEnd()) {
                return (*dataIt_)->getTimestamp();
            }
            return std::numeric_limits<uint64_t>::max();
        }

        std::shared_ptr<DataModels::GenericDataModel> LidarDataLoader::getNextData() {
            if (!isOnEnd()) {
                auto output = *dataIt_;
                dataIt_ = std::next(dataIt_,1);
                return output;
            }
            return std::make_shared<DataModels::ErrorDataModel>();
        }

        std::string LidarDataLoader::toString() {
            std::stringstream ss;
            ss << "[Lidar Data Loader] : size = " << data_.size();
            return ss.str();
        }

        uint64_t LidarDataLoader::getDataSize() {
            return data_.size();
        }

        bool LidarDataLoader::isOnEnd() {

            return dataIt_ >= data_.end();
        }

        void LidarDataLoader::setPose(timestamp_type timestamp) {
            for (dataIt_ = data_.begin(); dataIt_< data_.end() ; dataIt_ = std::next(dataIt_,1)) {
                if((*dataIt_)->getTimestamp() >= timestamp) {
                    break;
                }
            }
        }

        void LidarDataLoader::releaseOldData(timestamp_type keepHistory) {
            auto currentTime = (*dataIt_)->getTimestamp();
            while(releaseIt_ < data_.end()) {
                auto dataTimestamp = (*releaseIt_)->getTimestamp();
                if(dataTimestamp + keepHistory < currentTime) {
                    (*releaseIt_) = std::make_shared<DataModels::LidarScanDataModel>(0, LidarIdentifier::kNone, "", 0);
                    releaseIt_++;
                } else {
                    break;
                }
            }
        }

        void LidarDataLoader::clear() {
            data_.clear();
            dataIt_ = data_.begin();
        }

    }
}