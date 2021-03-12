/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include "DataModelTypes.h"

namespace AtlasFusion::DataModels {

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