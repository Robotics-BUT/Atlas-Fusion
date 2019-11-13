#pragma once

#include <vector>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "data_loader/data_models/GenericDataModel.h"


namespace AutoDrive {
    namespace DataLoader {

        class LidarScanDataModel : public GenericDataModel {

        public:

            LidarScanDataModel(uint64_t timestamp, LidarIdentifier id, std::string scan_path, uint64_t lidarTimestamp)
                    : GenericDataModel(timestamp)
                    , identifier_(id)
                    , scan_path_(scan_path)
                    , innerLidarTimestamp_(lidarTimestamp){
                type_ = DataModelTypes::kLidarScanDataModelType;
            };

            std::string toString() override;
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getScan();
            uint64_t getInnerTimestamp() {return innerLidarTimestamp_;};
            LidarIdentifier getLidarIdentifier() {return identifier_;};

        private:

            LidarIdentifier identifier_;
            pcl::PointCloud<pcl::PointXYZ> scan_{};
            std::string scan_path_;
            uint64_t innerLidarTimestamp_;
        };
    }
}
