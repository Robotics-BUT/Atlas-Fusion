#pragma once

#include "data_models/GenericDataModel.h"


namespace AutoDrive::DataModels {

    /**
     * Pressure Data Model represents atmospheric pressure measured in the given time
     */
    class ImuPressureDataModel : public GenericDataModel {

    public:

        /**
         * Constrcutor
         * @param timestamp pressure measurement time
         * @param pressure pressure value
         */
        ImuPressureDataModel(uint64_t timestamp, uint32_t pressure)
        : GenericDataModel(timestamp)
        , pressure_(pressure) {
            type_ = DataModelTypes::kImuPressDataModelType;
        };

        std::string toString() override;

        /**
         * Pressure value getter
         * @return atmospheric pressure value
         */
        uint32_t getPressure() {return pressure_;};

    private:

        uint32_t pressure_;

    };
}
