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

#include "data_loader/DataLoader.h"
#include "data_loader/RecordingConstants.h"
#include "Timer.h"

#include <iostream>
#include <experimental/filesystem>
#include <local_map/Frames.h>

namespace AutoDrive::DataLoader {

    bool DataLoader::loadData(const std::string &path) {

        if (!std::experimental::filesystem::is_directory(path)) {
            context_.logger_.error("Unable to open recording on path: " + path);
            return false;
        }

        auto isConsistent = checkRecordConsistency(path);
        if (!isConsistent) {
            context_.logger_.error("Recording is unconsistent");
            return false;
        }

        clear();
        auto isOk = loadRecord(path);

        return isOk;
    }


    timestamp_type DataLoader::getLowestTimestamp() {
        uint64_t minTimestamp = std::numeric_limits<uint64_t>::max();

        for (const auto &dataLoader: dataLoaders_) {
            if (dataLoader->getLowestTimestamp() < minTimestamp) {
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
        for (const auto &dataLoader: dataLoaders_) {
            sum += dataLoader->getDataSize();
        }
        return sum;
    }


    bool DataLoader::isOnEnd() {
        for (const auto &dataLoader: dataLoaders_) {
            if (!dataLoader->isOnEnd()) {
                return false;
            }
        }
        return true;
    }

    void DataLoader::setPose(timestamp_type pose) {
        for (const auto &dataLoader: dataLoaders_) {
            dataLoader->setPose(pose);
        }
    }

    std::shared_ptr<DataModels::GenericDataModel> DataLoader::getNextData() {
        Timer t("Get next data");
        AbstractDataLoader *it = nullptr;
        uint64_t minTimestamp = std::numeric_limits<uint64_t>::max();

        // Pick the nearest dataframe
        for (const auto &dataLoader: dataLoaders_) {
            if (dataLoader->getLowestTimestamp() < minTimestamp) {
                minTimestamp = dataLoader->getLowestTimestamp();
                it = dataLoader;
            }
        }

        if (it != nullptr) {
            auto ret = it->getNextData();
            it->releaseOldData(keepHistoryLength_);
            return ret;
        }

        throw std::runtime_error("Dataloader was null during data fetching");
    }


    void DataLoader::clear() {
        for (const auto &dataLoader: dataLoaders_) {
            dataLoader->clear();
        }
    }


    void DataLoader::releaseOldData(timestamp_type keepHistory) {
        for (const auto &dataLodar: dataLoaders_) {
            dataLodar->releaseOldData(keepHistory);
        }
    }


    bool DataLoader::checkRecordConsistency(const std::string &path) {

        for (const auto &folder: Folders::mandatoryFolders) {
            const auto directory = path + folder;
            if (!std::experimental::filesystem::is_directory(directory)) {
                context_.logger_.error("Unable to find mandatory folder: " + directory);
                return false;
            }
        }

        for (const auto &file: Files::mandatoryFiles) {
            const auto f = path + file;
            if (!std::experimental::filesystem::is_regular_file(f)) {
                context_.logger_.error("Unable to find mandatory file: " + f);
                return false;
            }
        }

        return true;
    }

    bool DataLoader::loadRecord(const std::string &path) {

        context_.logger_.info("Loading data ...");

        for (const auto &loader: dataLoaders_) {
            context_.threadPool_.push_task([&]() {
                auto result = loader->loadData(path);
                context_.logger_.info(loader->toString());
                if (!result) {
                    context_.logger_.error("Error when reading data from filesystem");
                    throw std::runtime_error("Error when reading data from filesystem");
                }
            });
        }
        context_.threadPool_.wait_for_tasks();

        context_.logger_.info("Data Loading done");
        return true;
    }


    CameraIndentifier DataLoader::getCameraIDfromFrame(const std::string &frame) {
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

    DataModels::CameraCalibrationParamsDataModel DataLoader::getCameraCalibDataForCameraID(CameraIndentifier id) {
        for (const auto &cameraDataLoader: cameraDataLoaders_) {
            if (cameraDataLoader->getCameraIdentifier() == id) {
                return cameraDataLoader->getCameraCalibParams();
            }
        }
        throw std::runtime_error("Couldn't get camera calibration data: " + std::to_string((int) id));
    }
}