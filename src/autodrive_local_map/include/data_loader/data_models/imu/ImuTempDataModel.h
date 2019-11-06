#pragma once

#include "data_loader/data_models/GenericDataModel.h"

namespace AutoDrive {
    namespace DataLoader {

        class ImuTempDataModel : public GenericDataModel {

        public:

            ImuTempDataModel(uint64_t timestamp, double temp)
            : GenericDataModel(timestamp)
            , temperature_(temp) {
                type_ = DataModelTypes::kImuTempDataModelType;
            };

            std::string toString() override;

            double getTemperature() {return temperature_;};

        private:

            double temperature_;

        };
    }
}
