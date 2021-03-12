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

#include "data_models/GenericDataModel.h"
#include "data_models/all.h"

#include <memory>
#include <vector>

namespace AtlasFusion {
    namespace DataLoader {

        typedef uint64_t timestamp_type;


        /**
         * Abstract Data Loader defines the basic API for each Data Loader class that handles specific sensor.
         */
        class AbstractDataLoader {

        public:

            /**
             * Load offline data stored on given path.
             * @param path to offline data
             * @return true if data have been loaded
             */
            virtual bool loadData(std::string path) = 0;

            /**
             * Method search trough the data and find the one with the lowest timestamp
             * @return lowest timestamp in data
             */
            virtual timestamp_type getLowestTimestamp() = 0;

            /**
             * Method returns the data frame with the lowest time stamp
             * @return the earliest data frame currently available
             */
            virtual std::shared_ptr<DataModels::GenericDataModel> getNextData() = 0;

            /**
             * Method creates short string description of the Data Loader class that describes data types, that Loader
             * handles and the number of loaded data frames.
             * @return string description of the Data Loader class
             */
            virtual std::string toString() = 0;

            /**
             * Method gives the total number of load data frames
             * @return number of data frames loaded by Data Loader
             */
            virtual uint64_t getDataSize() = 0;

            /**
             * Method says is the Data Loader is out of preloaded data frames
             * @return true, is no mo data available
             */
            virtual bool isOnEnd() = 0;

            /**
             * Method sets the positions in the loaded data on a frame that is equal or just after the given timestamp.
             * @param pose timestamp position on which the data loader should be setted up
             */
            virtual void setPose(timestamp_type pose) = 0;

            /**
             * Method clears the Data Loader memory and releases all the preloaded data
             */
            virtual void clear() = 0;

            /**
             * Method removed the data that are behind the current Data Loader time position with a interval of
             * tolerance.
             * @param historyLenght the tolerance period in nanoseconds in which even the old data are kept.
             */
            virtual void releaseOldData(timestamp_type historyLenght) = 0;

        protected:

            std::vector<std::vector<std::string>> readCsv(const std::string&& path) const;
            std::vector<std::string> split(const std::string& s, char delimiter) const;
        };

    }
}

