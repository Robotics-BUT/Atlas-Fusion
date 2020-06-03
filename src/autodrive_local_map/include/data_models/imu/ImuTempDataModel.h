#pragma once

#include "data_models/GenericDataModel.h"

namespace AutoDrive::DataModels {

    /**
     * IMU's internall temperature data model
     */
    class ImuTempDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp Time stamp when the temprerature has been measured
         * @param temp value of the temperature
         */
        ImuTempDataModel(uint64_t timestamp, double temp)
        : GenericDataModel(timestamp)
        , temperature_(temp) {
            type_ = DataModelTypes::kImuTempDataModelType;
        };

        std::string toString() override;

        /**
         * Temperature value getter
         * @return temperature
         */
        double getTemperature() {return temperature_;};

    private:

        double temperature_;

    };
}
