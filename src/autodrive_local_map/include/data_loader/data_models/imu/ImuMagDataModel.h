#pragma once

#include "data_loader/data_models/GenericDataModel.h"

#include <rtl/Vector3D.h>

namespace AutoDrive {
    namespace DataLoader {


        class ImuMagDataModel : public GenericDataModel {

        public:

            ImuMagDataModel(uint64_t timestamp, rtl::Vector3D<double> mag)
            : GenericDataModel(timestamp)
            , mag_(mag){
                type_ = DataModelTypes::kImuMagDataModelType;
            };

            std::string toString() override;

            rtl::Vector3D<double> getMagneticField() {return mag_;};

        private:

            rtl::Vector3D<double> mag_;

        };
    }
}
