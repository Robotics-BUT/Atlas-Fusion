#pragma once

#include "GenericDataModel.h"

namespace AutoDrive::DataModels{

    class ErrorDataModel : public GenericDataModel {

    public:

        ErrorDataModel()
                : GenericDataModel(0) {
            type_ = DataModelTypes::kErrorDataModelType;
        };

        std::string toString() override { return "Error Data Model";};

    };
}