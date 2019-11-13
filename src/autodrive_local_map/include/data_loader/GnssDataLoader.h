#pragma once

#include "AbstractDataLoader.h"
#include "data_models/all.h"

namespace AutoDrive {
    namespace DataLoader {

        class GnssDataLoader : public AbstractDataLoader{

        public:

            GnssDataLoader(GnssLoaderIdentifier id)
            : dataLoaderIdentifier_(id) {
                data_.clear();
                dataIt_ = data_.begin();
            }

            bool loadData(std::string path) override;
            uint64_t getLowestTimestamp() override;
            std::shared_ptr<GenericDataModel> getNextData() override;
            std::string toString() override;
            uint64_t getDataSize() override;
            bool isOnEnd() override;
            void setPose(timestamp_type) override;
            void releaseOldData(timestamp_type keepHistory) override;
            void clear() override;

        private:

            GnssLoaderIdentifier dataLoaderIdentifier_;

            std::vector<std::shared_ptr<GenericDataModel>> data_;
            std::vector<std::shared_ptr<GenericDataModel>>::iterator dataIt_;
            std::vector<std::shared_ptr<GenericDataModel>>::iterator releaseIt_;

            std::vector<std::shared_ptr<GenericDataModel>> loadGnssTimeData(std::string& path);
            std::vector<std::shared_ptr<GenericDataModel>> loadGnssPoseData(std::string& path);

        };

    }
}
