#pragma once

#include <iostream>
#include "AbstractDataLoader.h"
#include "data_models/DataModelTypes.h"
#include "data_models/all.h"

namespace AutoDrive {
    namespace DataLoader {

        class LidarDataLoader : public AbstractDataLoader{


        public:

            LidarDataLoader(LidarIdentifier id)
            : lidarIdentifier_(id){
                    data_.clear();
                    dataIt_ = data_.begin();
            }

            bool loadData(std::string path) override;
            timestamp_type getLowestTimestamp() override;
            std::shared_ptr<GenericDataModel> getNextData() override;
            std::string toString() override;
            uint64_t getDataSize() override;
            bool isOnEnd() override;
            void rewind() override;
            void setPose(timestamp_type) override;
            void clear() override;

        private:

            LidarIdentifier lidarIdentifier_;
            std::vector<std::shared_ptr<LidarScanDataModel>> data_;
            std::vector<std::shared_ptr<LidarScanDataModel>>::iterator dataIt_;

        };
    }
}
