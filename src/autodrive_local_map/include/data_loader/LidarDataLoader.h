#pragma once

#include <iostream>
#include "AbstractDataLoader.h"
#include "data_models/DataModelTypes.h"
#include "data_models/all.h"
#include "Context.h"

namespace AutoDrive {
    namespace DataLoader {

        /**
         * Lidar Data Loader handles the point cloud scans lazy loading and providing them on demand to the upper
         * Data Loader instance.
         */
        class LidarDataLoader : public AbstractDataLoader{

        public:

            /**
             *
             * @param context global services container (timestamps, logging, etc.)
             * @param id currently identifies the left or right LiDAR sensor
             */
            LidarDataLoader(Context& context, LidarIdentifier id)
            : context_{context}
            , lidarIdentifier_(id){
                data_.clear();
                dataIt_ = data_.begin();
            }

            bool loadData(std::string path) override;
            timestamp_type getLowestTimestamp() override;
            std::shared_ptr<DataModels::GenericDataModel> getNextData() override;
            std::string toString() override;
            uint64_t getDataSize() override;
            bool isOnEnd() override;
            void setPose(timestamp_type) override;
            void releaseOldData(timestamp_type keepHistory) override;
            void clear() override;

        private:
            Context& context_;
            LidarIdentifier lidarIdentifier_;
            std::vector<std::shared_ptr<DataModels::LidarScanDataModel>> data_;
            std::vector<std::shared_ptr<DataModels::LidarScanDataModel>>::iterator dataIt_;
            std::vector<std::shared_ptr<DataModels::LidarScanDataModel>>::iterator releaseIt_;

        };
    }
}
