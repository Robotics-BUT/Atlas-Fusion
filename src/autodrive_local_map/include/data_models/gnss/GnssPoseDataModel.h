#pragma once

#include "data_models/GenericDataModel.h"

namespace AutoDrive::DataModels {

    class GnssPoseDataModel : public GenericDataModel {

    public:

        GnssPoseDataModel(uint64_t timestamp, double lat, double lon, double alt, double azim)
        : GenericDataModel(timestamp)
        , latitude_(lat)
        , longitude_(lon)
        , altitude_(alt)
        , azimut_(azim) {
            type_ = DataModelTypes::kGnssPositionDataModelType;
        }

        std::string toString() override;

        double getLatitude() {return latitude_;};
        double getLongitude() {return longitude_;};
        double getAltitude() {return altitude_;};
        double getAzimut() {return azimut_;};

    private:

        double latitude_;
        double longitude_;
        double altitude_;
        double azimut_;
    };
}