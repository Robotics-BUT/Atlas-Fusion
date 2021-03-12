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

#include "data_loader/ImuDataLoader.h"

#include <iostream>
#include <limits>

namespace AtlasFusion {
    namespace DataLoader {


        bool ImuDataLoader::loadData(std::string path) {

            switch(dataLoaderIdentifier_) {
                case ImuLoaderIdentifier::kDQuat:
                    data_ = loadImuDquatData(path); break;
                case ImuLoaderIdentifier::kGnss:
                    data_ = loadImuGnssData(path); break;
                case ImuLoaderIdentifier::kImu:
                    data_ = loadImuImuData(path); break;
                case ImuLoaderIdentifier::kMag:
                    data_ = loadImuMagData(path); break;
                case ImuLoaderIdentifier::kPressure:
                    data_ = loadImuPressureData(path); break;
                case ImuLoaderIdentifier::kTemp:
                    data_ = loadImuTempData(path); break;
                case ImuLoaderIdentifier::kTime:
                    data_ = loadImuTimeData(path); break;
            }
            dataIt_ = data_.begin();
            releaseIt_ = dataIt_;
            return true;
        }

        timestamp_type ImuDataLoader::getLowestTimestamp() {
            if(!isOnEnd()) {
                return (*dataIt_)->getTimestamp();
            }
            return std::numeric_limits<uint64_t>::max();
        }

        std::shared_ptr<DataModels::GenericDataModel> ImuDataLoader::getNextData() {
            if (!isOnEnd()) {
                auto output = *dataIt_;
                dataIt_ = std::next(dataIt_,1);
                return output;
            }
            return std::make_shared<DataModels::ErrorDataModel>();
        }

        std::string ImuDataLoader::toString() {
            std::stringstream ss;
            std::string type;

            switch(dataLoaderIdentifier_) {
                case ImuLoaderIdentifier::kDQuat:
                    type = "DQuat"; break;
                case ImuLoaderIdentifier::kGnss:
                    type = "Gnss"; break;
                case ImuLoaderIdentifier::kImu:
                    type = "Imu"; break;
                case ImuLoaderIdentifier::kMag:
                    type = "Mag"; break;
                case ImuLoaderIdentifier::kPressure:
                    type = "Pressure"; break;
                case ImuLoaderIdentifier::kTemp:
                    type = "Temp"; break;
                case ImuLoaderIdentifier::kTime:
                    type = "Time"; break;
            }

            ss << "[Imu " << type << " Data Loader] : " << data_.size();
            return ss.str();
        }

        uint64_t ImuDataLoader::getDataSize() {
            return data_.size();
        }

        bool ImuDataLoader::isOnEnd() {

            return dataIt_ >= data_.end();
        }

        void ImuDataLoader::setPose(timestamp_type timestamp) {
            for (dataIt_ = data_.begin(); dataIt_< data_.end() ; dataIt_ = std::next(dataIt_,1)) {
                if((*dataIt_)->getTimestamp() >= timestamp) {
                    break;
                }
            }
        }


        void ImuDataLoader::releaseOldData(timestamp_type keepHistory) {
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

        void ImuDataLoader::clear() {
            data_.clear();
            dataIt_ = data_.begin();
        }


        std::vector<std::shared_ptr<DataModels::GenericDataModel>> ImuDataLoader::loadImuDquatData(std::string& path) {
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kImuFolder + Files::kDquatFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 5) {
                    output.push_back( std::make_shared<DataModels::ImuDquatDataModel>(std::stoll(substrings[0]),
                                                                          rtl::Quaternion<double>{std::stod(substrings[4]),
                                                                                                  std::stod(substrings[1]),
                                                                                                  std::stod(substrings[2]),
                                                                                                  std::stod(substrings[3])}
                    ));
                } else {
                    context_.logger_.error("Unexpected lenght of imu d_quat data:");
                }
            }
            return output;
        }

        std::vector<std::shared_ptr<DataModels::GenericDataModel>> ImuDataLoader::loadImuGnssData(std::string& path) {
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kImuFolder + Files::kGnssFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 4) {
                    output.push_back( std::make_shared<DataModels::ImuGnssDataModel>(
                            std::stoll(substrings[0]),std::stod(substrings[1]),
                            std::stod(substrings[2]),std::stod(substrings[3])));
                } else {
                    context_.logger_.error("Unexpected lenght of imu gnss data: ");
                }
            }
            return output;
        }

        std::vector<std::shared_ptr<DataModels::GenericDataModel>> ImuDataLoader::loadImuImuData(std::string& path) {
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kImuFolder + Files::kImuFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 11) {
                    output.push_back( std::make_shared<DataModels::ImuImuDataModel>(std::stoll(substrings[0]),
                                                                        rtl::Vector3D<double>{-std::stod(substrings[1]),
                                                                                              -std::stod(substrings[2]),
                                                                                              -std::stod(substrings[3])},
                                                                        rtl::Vector3D<double>{std::stod(substrings[4]),
                                                                                              std::stod(substrings[5]),
                                                                                              std::stod(substrings[6])},
                                                                        rtl::Quaternion<double>{std::stod(substrings[10]),
                                                                                                std::stod(substrings[7]),
                                                                                                std::stod(substrings[8]),
                                                                                                std::stod(substrings[9])}));
                } else {
                    context_.logger_.error("Unexpected lenght of imu imu data: ");
                }
            }
            return output;
        }

        std::vector<std::shared_ptr<DataModels::GenericDataModel>> ImuDataLoader::loadImuMagData(std::string& path) {
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kImuFolder + Files::kMagFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 4) {
                    output.push_back( std::make_shared<DataModels::ImuMagDataModel>(std::stoll(substrings[0]),
                                                                        rtl::Vector3D<double>{std::stod(substrings[1]),
                                                                                              std::stod(substrings[2]),
                                                                                              std::stod(substrings[3])}));
                } else {
                    context_.logger_.error("Unexpected lenght of imu mag data: ");
                }
            }
            return output;
        }

        std::vector<std::shared_ptr<DataModels::GenericDataModel>> ImuDataLoader::loadImuPressureData(std::string& path) {
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kImuFolder + Files::kPressureFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 2) {
                    output.push_back( std::make_shared<DataModels::ImuPressureDataModel>(std::stoll(substrings[0]),std::stoll(substrings[1])));
                } else {
                    context_.logger_.error("Unexpected lenght of imu pressure data: ");
                }
            }
            return output;
        }

        std::vector<std::shared_ptr<DataModels::GenericDataModel>> ImuDataLoader::loadImuTempData(std::string& path) {
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kImuFolder + Files::kTempFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 2) {
                    output.push_back( std::make_shared<DataModels::ImuTempDataModel>(std::stoll(substrings[0]),std::stod(substrings[1])));
                } else {
                    context_.logger_.error("Unexpected lenght of imu temp data: ");
                }
            }
            return output;
        }

        std::vector<std::shared_ptr<DataModels::GenericDataModel>> ImuDataLoader::loadImuTimeData(std::string& path) {
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> output;
            auto csvContent = readCsv(path + Folders::kImuFolder + Files::kTimeFile);
            for(const auto& substrings : csvContent) {
                if(substrings.size() == 8) {
                    output.push_back( std::make_shared<DataModels::ImuTimeDataModel>(std::stoll(substrings[0]),std::stoll(substrings[1]),
                                                                         std::stoll(substrings[2]),std::stoll(substrings[3]),
                                                                         std::stoll(substrings[4]),std::stoll(substrings[5]),
                                                                         std::stoll(substrings[6]),std::stoll(substrings[7])));
                } else {
                    context_.logger_.error("Unexpected lenght of imu time data: ");
                }
            }
            return output;
        }
    }
}