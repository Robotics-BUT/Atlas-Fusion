#pragma once

#include "GenericDataModel.h"

namespace AutoDrive::DataModels{

    /**
     * Error Data Model is used for representation of the missing data or other unexpected event.
     */
    class ErrorDataModel : public GenericDataModel {

    public:

        ErrorDataModel()
                : GenericDataModel(0) {
            type_ = DataModelTypes::kErrorDataModelType;
        };

        std::string toString() override { return "Error Data Model";};

    };
}