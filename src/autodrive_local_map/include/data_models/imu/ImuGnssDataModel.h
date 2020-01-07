#pragma once

#include "data_models/GenericDataModel.h"

namespace AutoDrive::DataModels {

    class ImuGnssDataModel : public GenericDataModel {

    public:

        ImuGnssDataModel(uint64_t timestamp, double lat, double lon, double alt)
        : GenericDataModel(timestamp)
        , latitude_(lat)
        , longitude_(lon)
        , altitude_(alt) {
            type_ = DataModelTypes::kImuGnssDataModelType;
        };

        std::string toString() override;

        double getLatitude() {return latitude_;};
        double getLongitude() {return longitude_;};
        double getAltitude() {return altitude_;};

    private:

        double latitude_;
        double longitude_;
        double altitude_;

    };
}

