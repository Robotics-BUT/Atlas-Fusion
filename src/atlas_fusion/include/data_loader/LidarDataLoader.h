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

#include <iostream>
#include "AbstractDataLoader.h"
#include "data_models/DataModelTypes.h"
#include "data_models/all.h"
#include "Context.h"

namespace AtlasFusion {
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
