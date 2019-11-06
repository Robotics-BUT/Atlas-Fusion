#pragma once

#include "data_loader/data_models/GenericDataModel.h"


namespace AutoDrive {
    namespace DataLoader {

        class ImuPressureDataModel : public GenericDataModel {

        public:

            ImuPressureDataModel(uint64_t timestamp, uint32_t pressure)
            : GenericDataModel(timestamp)
            , pressure_(pressure) {
                type_ = DataModelTypes::kImuPressDataModelType;
            };

            std::string toString() override;

            uint32_t getPressure() {return pressure_;};

        private:

            uint32_t pressure_;

        };
    }
}
