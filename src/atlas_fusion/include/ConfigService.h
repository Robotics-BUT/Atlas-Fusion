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
#include <yaml-cpp/yaml.h>

namespace AtlasFusion {

    /**
     *  Config Service is a class used to read out the startup configuration from the yaml file.
     */

    class ConfigService {

    private:


    public:

        ConfigService(std::string&& configPath)
        : confPath_(configPath) {

        };

        /**
         * Method extracts the uint16_t value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return uint16_t value that is corresponding to the given keys
         */
        uint16_t getUInt16Value(const std::vector<std::string>& keys) { return getNode(keys).as<uint16_t >(); }

        /**
         * Method extracts the uint32_t value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return uint32_t value that is corresponding to the given keys
         */
        uint32_t getUInt32Value(const std::vector<std::string>& keys) { return getNode(keys).as<uint32_t >(); }

        /**
         * Method extracts the int32_t value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return int32_t value that is corresponding to the given keys
         */
        int32_t getInt32Value(const std::vector<std::string>& keys) { return getNode(keys).as<int32_t >(); }

        /**
         * Method extracts the float value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return float value that is corresponding to the given keys
         */
        float getFloatValue(const std::vector<std::string>& keys) { return getNode(keys).as<float >(); }

        /**
         * Method extracts the double value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return double value that is corresponding to the given keys
         */
        double getDoubleValue(const std::vector<std::string>& keys) { return getNode(keys).as<double >(); }

        /**
         * Method extracts the string value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return string value that is corresponding to the given keys
         */
        std::string getStringValue(const std::vector<std::string>& keys) { return getNode(keys).as<std::string >(); }

        /**
         * Method extracts the bool value that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return bool value that is corresponding to the given keys
         */
        bool getBoolValue(const std::vector<std::string>& keys) {
            const auto value = getStringValue(keys);
            if (value == "True" || value == "true" || value == "1") {
                return true;
            }
            return false;
        }

        /**
         * Method extracts the the array of values, represented as a std::vector. Type is defined via the template.
         * @param keys Vector of keys that are used to access the values.
         * @return vector of values that is corresponding to the given keys
         */
        template <typename T>
        std::vector<T> getArrayValue(const std::vector<std::string>& keys) {
            auto node = getNode(keys).as<std::vector<T> >();
            return node;
        }

        /**
         * Method extracts the 4-element quaternion that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the quaternion.
         * @return instance of the rtl::Quaternion that is corresponding to the given keys
         */
        template <typename T>
        rtl::Quaternion<T> getQuaternionValue(const std::vector<std::string>& keys) {
            auto arr = getArrayValue<T>(keys);
            return rtl::Quaternion<T>{arr[3], arr[0], arr[1], arr[2]};
        }

        /**
         * Method extracts the vector of 3 values that correspond to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return instance of rhe rtl::Vector3D that is corresponding to the given keys
         */
        template <typename T>
        rtl::Vector3D<T> getVector3DValue(const std::vector<std::string>& keys) {
            auto arr = getArrayValue<T>(keys);
            return rtl::Vector3D<T>{ arr[0], arr[1], arr[2]};
        }

        /**
         * Method extracts the matrix that corresponds to the given keys, in the yaml file.
         * @param keys Vector of keys that are used to access the value.
         * @return the matrix of data types defined by the template, represented as a std::vec of std:vecs
         */
        template <typename T>
        std::vector<std::vector<T>> getMatValue(const std::vector<std::string>& keys) {
            auto arr = getArrayValue<T>(keys);
            size_t size = size_t(std::sqrt(arr.size()));

            std::vector<std::vector<T>> output;
            if( size == std::sqrt(arr.size())) {
                for(size_t i = 0 ; i < size ; i++) {
                    std::vector<T> vec;
                    for(size_t j = 0 ; j < size ; j++) {
                        vec.push_back(arr[j+i*size]);
                    }
                    output.push_back(vec);
                }
            }
            return output;
        }


    protected:

        const std::string confPath_;

        YAML::Node getNode(const std::vector<std::string>& keys) {
            YAML::Node config = YAML::LoadFile(confPath_);
            for (const auto& key : keys) {
                if (config[key]) {
                    config = config[key];
                } else {
                    std::cerr << "Unable to read \"" << key << "\" from config file!" << std::endl;
                    throw std::exception();
                }
            }
            return config;
        }
    };
}
