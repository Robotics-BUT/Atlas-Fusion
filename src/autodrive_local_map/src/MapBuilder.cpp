#include "MapBuilder.h"

namespace AutoDrive {

    void MapBuilder::buildMap() {
        algorithm_.doCoolStaff();
        map_.doMapping();
        visualizer_.visualizeSomething();
    }
}