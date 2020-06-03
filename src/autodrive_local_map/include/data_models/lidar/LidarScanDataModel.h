#pragma once

#include <vector>
#include <iostream>
#include <functional>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "data_models/GenericDataModel.h"
#include "data_loader/DataLoaderIdentifiers.h"


namespace AutoDrive::DataModels {

    /**
     * Lidar Scan Data Model is a wrapper around the LiDAR scan loaded as a raw data from the record session.
     */
    class LidarScanDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp nanosecond timestampo when the scan has been taken
         * @param id serial number of the scan
         * @param scan_path path to the scan stored on the disk
         * @param lidarTimestamp internal sensor's timestamp
         */
        LidarScanDataModel(uint64_t timestamp, DataLoader::LidarIdentifier id, std::string scan_path, uint64_t lidarTimestamp)
                : GenericDataModel(timestamp)
                , identifier_(id)
                , scan_path_(scan_path)
                , innerLidarTimestamp_(lidarTimestamp){
            type_ = DataModelTypes::kLidarScanDataModelType;
        };

        std::string toString() override;

        /**
         * Method lazy loads scan from the storage applies filters and returns it as a point cloud
         * @return point cloud of the scan
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getScan();

        /**
         * Method lazy loads unfiltered scan from the storage, returns it as a point cloud
         * @return unfiltered scan
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getRawScan();

        /**
         * Inner LiDAR timestamp
         * @return sensor's timestamp
         */
        uint64_t getInnerTimestamp() {return innerLidarTimestamp_;};

        /**
         * Get the left/right lidar identifier
         * @return unique identifier of the lidar scanner that has captured this scan
         */
        DataLoader::LidarIdentifier getLidarIdentifier() {return identifier_;};

        /**
         * Set up filtration function that will be applied on point cloud
         * @param fnc point cloud filtration function
         */
        void registerFilter(std::function<void(pcl::PointCloud<pcl::PointXYZ>&)> fnc) { filter_ = fnc; };

        /**
         * Add more points into the scan
         * @param point new points that will be added into the point cloud
         */
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
