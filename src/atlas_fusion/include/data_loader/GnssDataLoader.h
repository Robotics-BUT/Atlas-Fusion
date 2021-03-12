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

#pragma once

#include "AbstractDataLoader.h"
#include "data_models/all.h"
#include "Context.h"

namespace AtlasFusion {
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
