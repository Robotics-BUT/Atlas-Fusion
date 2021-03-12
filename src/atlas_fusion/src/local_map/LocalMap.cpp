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

#include "local_map/LocalMap.h"

namespace AtlasFusion::LocalMap {


    void LocalMap::setFrustumDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> detections, std::string sensorFrame) {
        frustumsDetections_[sensorFrame] = detections;
    }



    void LocalMap::setLidarDetections(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections) {
        lidarDetections_ = detections;
    }


    void LocalMap::setObjects(std::vector<std::shared_ptr<DataModels::Object>> objects) {
        objects_ = objects;
    }


    std::vector<std::shared_ptr<const DataModels::LidarDetection>> LocalMap::getObjectsAsLidarDetections() {

        std::vector<std::shared_ptr<const DataModels::LidarDetection>> output;

        return output;
    }


    std::vector<std::shared_ptr<const DataModels::FrustumDetection>> LocalMap::getFrustumDetections() {

        std::vector<std::shared_ptr<const DataModels::FrustumDetection>> output;
        for(auto it = frustumsDetections_.begin(); it != frustumsDetections_.end(); it++){
            for(auto& frustum : it->second) {
                output.push_back(frustum);
            }
        }
        return output;
    }



    std::vector<std::shared_ptr<const DataModels::LidarDetection>> LocalMap::getLidarDetections() {
        return lidarDetections_;
    }


    std::vector<std::shared_ptr<DataModels::Object>> LocalMap::getObjects() {
        return objects_;
    }

}