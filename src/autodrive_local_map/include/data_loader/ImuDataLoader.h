#pragma once

#include "AbstractDataLoader.h"
#include "RecordingConstants.h"
#include "Context.h"

namespace AutoDrive {
    namespace DataLoader {

        class ImuDataLoader : public AbstractDataLoader {

        public:

            ImuDataLoader(Context& context, ImuLoaderIdentifier id)
            : context_{context}
            , dataLoaderIdentifier_(id) {
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
            ImuLoaderIdentifier dataLoaderIdentifier_;

            std::vector<std::shared_ptr<DataModels::GenericDataModel>> data_;
            std::vector<std::shared_ptr<DataModels::GenericDataModel>>::iterator dataIt_;
            std::vector<std::shared_ptr<DataModels::GenericDataModel>>::iterator releaseIt_;

            std::vector<std::shared_ptr<DataModels::GenericDataModel>> loadImuDquatData(std::string& path);
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> loadImuGnssData(std::string& path);
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> loadImuImuData(std::string& path);
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> loadImuMagData(std::string& path);
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> loadImuPressureData(std::string& path);
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> loadImuTempData(std::string& path);
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> loadImuTimeData(std::string& path);
        };

    }
}
