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

namespace AutoDrive::DataModels {

    /**
     * Time Data Model that represents inner IMU time
     */
    class ImuTimeDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp system's timestamp
         * @param year
         * @param month
         * @param day
         * @param hour
         * @param minute
         * @param sec
         * @param nsec
         */
        ImuTimeDataModel(uint64_t timestamp, uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t sec, uint32_t nsec)
        : GenericDataModel(timestamp)
        , year_(year)
        , month_(month)
        , day_(day)
        , hour_(hour)
        , minute_(minute)
        , sec_(sec)
        , nsec_(nsec){
            type_ = DataModelTypes::kImuTimeDataModelType;
        }

        std::string toString() override;

        /**
         * Year getter
         * @return year
         */
        inline uint32_t getYear() { return year_; };

        /**
         * Month getter
         * @return month
         */
        inline uint32_t getMonth() { return month_; };

        /**
         * Day getter
         * @return day
         */
        inline uint32_t getDay() { return day_; };

        /**
         * Hour getter
         * @return hour
         */
        inline uint32_t getHour() { return hour_; };

        /**
         * Minute getter
         * @return minute
         */
        inline uint32_t getMinute() { return minute_; };

        /**
         * Second getter
         * @return sec
         */
        inline uint32_t getSec() { return sec_; };

        /**
         * Nanosecond getter
         * @return nsec
         */
        inline uint32_t getNSec() { return nsec_; };

    private:
        uint32_t year_;
        uint32_t month_;
        uint32_t day_;
        uint32_t hour_;
        uint32_t minute_;
        uint32_t sec_;
        uint32_t nsec_;
    };
}
