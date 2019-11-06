#pragma once

#include "algorithms/GenericAlgorithm.h"
#include "local_map/GenericLocalMap.h"
#include "visualizers/GenericVisializer.h"
#include "data_loader/DataLoader.h"

namespace AutoDrive {

    class MapBuilder {

    public:

        MapBuilder() = default;

        void buildMap();

    private:

        Algorithms::GenericAlgorithm algorithm_;
        LocalMap::GenericLocalMap map_;
        Visualizers::GenericVisualizer visualizer_;

        DataLoader::DataLoader dataLoader_{};

    };

}