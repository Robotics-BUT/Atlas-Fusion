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
