#pragma once

#include "data_models/GenericDataModel.h"

namespace AutoDrive::DataModels {

    class ImuTimeDataModel : public GenericDataModel {

    public:

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

        inline uint32_t getYear() { return year_; };
        inline uint32_t getMonth() { return month_; };
        inline uint32_t getDay() { return day_; };
        inline uint32_t getHour() { return hour_; };
        inline uint32_t getMinute() { return minute_; };
        inline uint32_t getSec() { return sec_; };
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
