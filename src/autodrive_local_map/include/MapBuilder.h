#pragma once

#include <ros/ros.h>

#include "algorithms/GenericAlgorithm.h"
#include "local_map/GenericLocalMap.h"
#include "visualizers/GenericVisializer.h"
#include "data_loader/DataLoader.h"
#include "visualizers/VisualizationHandler.h"

#include "LogService.h"
#include "TFTree.h"

namespace AutoDrive {

    class MapBuilder {

    public:

        explicit MapBuilder(ros::NodeHandle& n, TFTree& tfTree, LogService& logger, DataLoader::timestamp_type keepHistoryLength, double maxReplayerRate)
        : rosNode_{n}
        , tfTree_{tfTree}
        , visualizationHandler_(rosNode_, tfTree, logger)
        , logger_(logger)
        , dataLoader_(logger, keepHistoryLength)
        , keepHistoryLength_(keepHistoryLength)
        , maxReplayerRate_(maxReplayerRate) {

        }

        void loadData(const std::string&);
        void buildMap();
        void clearData();

    private:

        ros::NodeHandle& rosNode_;
        TFTree& tfTree_;

        Algorithms::GenericAlgorithm algorithm_;
        LocalMap::GenericLocalMap map_;
        Visualizers::GenericVisualizer visualizer_;
        Visualizers::VisualizationHandler visualizationHandler_;
        LogService& logger_;

        DataLoader::DataLoader dataLoader_;
        DataLoader::timestamp_type keepHistoryLength_;
        double maxReplayerRate_;

    };

}