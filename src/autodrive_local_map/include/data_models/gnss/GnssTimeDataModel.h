#pragma once

#include "data_models/GenericDataModel.h"

namespace AutoDrive::DataModels {

    /**
     * GNSS time frame data received by the GNSS Receiver
     */
    class GnssTimeDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp recording session timestamp
         * @param year GNSS time - year
         * @param month GNSS time - month
         * @param day GNSS time - day
         * @param hour GNSS time - hour
         * @param minute GNSS time - minutes
         * @param sec GNSS time - seconds
         * @param nsec GNSS time - nanoseconds
         */
        GnssTimeDataModel(uint64_t timestamp, uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t sec, uint32_t nsec)
        : GenericDataModel(timestamp)
        , year_(year)
        , month_(month)
        , day_(day)
        , hour_(hour)
        , minute_(minute)
        , sec_(sec)
        , nsec_(nsec) {
            type_ = DataModelTypes::kGnssTimeDataModelType;
        }

        std::string toString() override;

        /**
         * GNSS time - year getter
         * @return year
         */
        inline uint32_t getYear() { return year_; };

        /**
         * GNSS time - month getter
         * @return month
         */
        inline uint32_t getMonth() { return month_; };

        /**
         * GNSS time - day getter
         * @return day
         */
        inline uint32_t getDay() { return day_; };

        /**
         * GNSS time - hour getter
         * @return hour
         */
        inline uint32_t getHour() { return hour_; };

        /**
         * GNSS time - minutes getter
         * @return minutes
         */
        inline uint32_t getMinute() { return minute_; };

        /**
         * GNSS time - seconds getter
         * @return seconds
         */
        inline uint32_t getSec() { return sec_; };

        /**
         * GNSS time - nanoseconds getter
         * @return nanoseconds
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