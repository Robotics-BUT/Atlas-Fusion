#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include "DataModelTypes.h"

namespace AutoDrive {
    namespace DataLoader {

        class GenericDataModel {

        public:

            GenericDataModel(uint64_t timestamp)
            : timestamp_(timestamp)
            , type_(DataModelTypes::kGenericDataModelType) {

            };

            virtual std::string toString() { return "Generic Data Model";};
            DataModelTypes getType() { return type_; };
            uint64_t getTimestamp() { return timestamp_; };
            std::vector<std::shared_ptr<GenericDataModel>> getParents() {return parents_;};

        protected:

            const uint64_t timestamp_;
            DataModelTypes type_;
            std::vector<std::shared_ptr<GenericDataModel>> parents_{};
        };
    }
}