#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace AutoDrive {

    class ConfigService {


    private:


    public:

        ConfigService(std::string&& configPath)
        : confPath_(configPath) {

        };

        uint16_t getUInt16Value(const std::vector<std::string>& keys) { return getNode(keys).as<uint16_t >(); }
        uint32_t getUInt32Value(const std::vector<std::string>& keys) { return getNode(keys).as<uint32_t >(); }
        int32_t getInt32Value(const std::vector<std::string>& keys) { return getNode(keys).as<int32_t >(); }
        float getFloatValue(const std::vector<std::string>& keys) { return getNode(keys).as<float >(); }
        double getDoubleValue(const std::vector<std::string>& keys) { return getNode(keys).as<double >(); }
        std::string getStringValue(const std::vector<std::string>& keys) { return getNode(keys).as<std::string >(); }
        bool getBoolValue(const std::vector<std::string>& keys) { return getNode(keys).as<bool >(); }

        template <typename T>
        std::vector<T> getArrayValue(const std::vector<std::string>& keys) {
            auto node = getNode(keys).as<std::vector<T> >();
            return node;
        }

        template <typename T>
        rtl::Quaternion<T> getQuaternionValue(const std::vector<std::string>& keys) {
            auto arr = getArrayValue<T>(keys);
            return rtl::Quaternion<T>{arr[3], arr[0], arr[1], arr[2]};
        }

        template <typename T>
        rtl::Vector3D<T> getVector3DValue(const std::vector<std::string>& keys) {
            auto arr = getArrayValue<T>(keys);
            return rtl::Vector3D<T>{ arr[0], arr[1], arr[2]};
        }

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
