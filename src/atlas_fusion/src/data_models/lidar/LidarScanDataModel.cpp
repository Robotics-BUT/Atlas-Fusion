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

#include "data_models/lidar/LidarScanDataModel.h"

#include <sstream>

namespace AtlasFusion::DataModels {

    std::string LidarScanDataModel::toString() {

        std::stringstream ss;
        ss << "[Lidar Scan Data Model] : "
           << scan_path_ << scan_.size();
        return ss.str();
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> LidarScanDataModel::getScan() {
        if(filteredScan_.points.empty()) {
            if(scan_.points.empty()) {
                getRawScan();
            }
            filteredScan_ = scan_;
            filter_(filteredScan_);
        }

        return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(filteredScan_);
    }


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> LidarScanDataModel::getRawScan() {
        if(scan_.points.empty()) {
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_path_, scan_) == -1) {
                std::cerr << "Could not open pcd file: " << scan_path_ << std::endl;
            }
        }
        return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(scan_);
    }

    void LidarScanDataModel::addPointToScan(pcl::PointXYZ point) {
        scan_.push_back(point);
    };
}
