#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include "DataModelTypes.h"

namespace AutoDrive::DataModels {

    /**
     * Generic Data Model works as a base class for all the data model classes.
     */
    class GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp nanosecond timestamp that identifies time position of the data
         */
        GenericDataModel(uint64_t timestamp)
        : timestamp_(timestamp)
        , type_(DataModelTypes::kGenericDataModelType) {

        };

        /**
         * Method creates simple string description of the class intance
         * @return string description of the data type
         */
        virtual std::string toString() { return "Generic Data Model";};

        /**
         * Method returns the data identifier of a data model. Used for up/down casting of the data models
         * @return Data Model type identifier
         */
        DataModelTypes getType() { return type_; };

        /**
         * Method returns the data model timestamp
         * @return nanosecond timestamp
         */
        uint64_t getTimestamp() { return timestamp_; };

        /**
         * Returns all the data models (data) that are related to this one
         * @return vector of related data models
         */
        std::vector<std::shared_ptr<GenericDataModel>> getParents() {return parents_;};

    protected:

        const uint64_t timestamp_;
        DataModelTypes type_;
        std::vector<std::shared_ptr<GenericDataModel>> parents_{};
    };
}