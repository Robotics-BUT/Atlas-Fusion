#pragma once

#include <vector>
#include <iostream>
#include <functional>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "data_models/GenericDataModel.h"
#include "data_loader/DataLoaderIdentifiers.h"


namespace AutoDrive::DataModels {

    class LidarScanDataModel : public GenericDataModel {

    public:

        LidarScanDataModel(uint64_t timestamp, DataLoader::LidarIdentifier id, std::string scan_path, uint64_t lidarTimestamp)
                : GenericDataModel(timestamp)
                , identifier_(id)
                , scan_path_(scan_path)
                , innerLidarTimestamp_(lidarTimestamp){
            type_ = DataModelTypes::kLidarScanDataModelType;
        };

        std::string toString() override;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getScan();
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getRawScan();
        uint64_t getInnerTimestamp() {return innerLidarTimestamp_;};
        DataLoader::LidarIdentifier getLidarIdentifier() {return identifier_;};
        void registerFilter(std::function<void(pcl::PointCloud<pcl::PointXYZ>&)> fnc) { filter_ = fnc; };

        void addPointToScan(pcl::PointXYZ point);

    private:

        DataLoader::LidarIdentifier identifier_;
        pcl::PointCloud<pcl::PointXYZ> scan_{};
        pcl::PointCloud<pcl::PointXYZ> filteredScan_{};
        std::string scan_path_;
        uint64_t innerLidarTimestamp_;
        std::function<void(pcl::PointCloud<pcl::PointXYZ>&)> filter_ = nullptr;
    };
}
