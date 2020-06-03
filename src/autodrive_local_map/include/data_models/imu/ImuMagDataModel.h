#pragma once

#include "data_models/GenericDataModel.h"

#include <rtl/Vector3D.h>

namespace AutoDrive::DataModels {

    /**
     * 3D magnetic field indensity data model measured by IMU
     */
    class ImuMagDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp measurement timestamp
         * @param mag 3D magnetic field intensity
         */
        ImuMagDataModel(uint64_t timestamp, rtl::Vector3D<double> mag)
        : GenericDataModel(timestamp)
        , mag_(mag){
            type_ = DataModelTypes::kImuMagDataModelType;
        };

        std::string toString() override;

        /**
         * 3D magnetic field intensity measurement getter
         * @return 3D vector of magnetic field intensity
         */
        rtl::Vector3D<double> getMagneticField() {return mag_;};

    private:

        rtl::Vector3D<double> mag_;

    };
}
