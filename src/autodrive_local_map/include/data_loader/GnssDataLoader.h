#pragma once

#include "AbstractDataLoader.h"
#include "data_models/all.h"
#include "Context.h"

namespace AutoDrive {
    namespace DataLoader {

        /**
         * GNSS Data Loader loads and handles data providing for the RTK GNNS Receiver sensor. It covers the global
         * positioning, heading estimation and GNSS timestamping.
         */
        class GnssDataLoader : public AbstractDataLoader{

        public:

            /**
             * Constructor
             * @param context global services container (timestamps, logging, etc.)
             * @param id distinguishes if the GNSS handles the time or the position data
             */
            GnssDataLoader(Context& context, GnssLoaderIdentifier id)
            : context_{context}
            , dataLoaderIdentifier_(id) {
                data_.clear();
                dataIt_ = data_.begin();
            }

            bool loadData(std::string path) override;
            uint64_t getLowestTimestamp() override;
            std::shared_ptr<DataModels::GenericDataModel> getNextData() override;
            std::string toString() override;
            uint64_t getDataSize() override;
            bool isOnEnd() override;
            void setPose(timestamp_type) override;
            void releaseOldData(timestamp_type keepHistory) override;
            void clear() override;

        private:

            Context& context_;
            GnssLoaderIdentifier dataLoaderIdentifier_;

            std::vector<std::shared_ptr<DataModels::GenericDataModel>> data_;
            std::vector<std::shared_ptr<DataModels::GenericDataModel>>::iterator dataIt_;
            std::vector<std::shared_ptr<DataModels::GenericDataModel>>::iterator releaseIt_;

            std::vector<std::shared_ptr<DataModels::GenericDataModel>> loadGnssTimeData(std::string& path);
            std::vector<std::shared_ptr<DataModels::GenericDataModel>> loadGnssPoseData(std::string& path);
        };

    }
}
