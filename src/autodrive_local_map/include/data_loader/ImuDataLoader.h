#pragma once

#include "AbstractDataLoader.h"
#include "RecordingConstants.h"

namespace AutoDrive {
    namespace DataLoader {

        class ImuDataLoader : public AbstractDataLoader {

        public:

            ImuDataLoader(ImuLoaderIdentifier id)
            : dataLoaderIdentifier_(id) {
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

            ImuLoaderIdentifier dataLoaderIdentifier_;

            std::vector<std::shared_ptr<GenericDataModel>> data_;
            std::vector<std::shared_ptr<GenericDataModel>>::iterator dataIt_;

            std::vector<std::shared_ptr<GenericDataModel>> loadImuDquatData(std::string& path);
            std::vector<std::shared_ptr<GenericDataModel>> loadImuGnssData(std::string& path);
            std::vector<std::shared_ptr<GenericDataModel>> loadImuImuData(std::string& path);
            std::vector<std::shared_ptr<GenericDataModel>> loadImuMagData(std::string& path);
            std::vector<std::shared_ptr<GenericDataModel>> loadImuPressureData(std::string& path);
            std::vector<std::shared_ptr<GenericDataModel>> loadImuTempData(std::string& path);
            std::vector<std::shared_ptr<GenericDataModel>> loadImuTimeData(std::string& path);
        };

    }
}
