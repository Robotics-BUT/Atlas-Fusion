#include "MapBuilder.h"
#include "data_loader/data_models/all.h"
#include "data_loader/data_models/DataModelTypes.h"

namespace AutoDrive {

    void MapBuilder::buildMap() {
        algorithm_.doCoolStaff();
        map_.doMapping();
        visualizer_.visualizeSomething();

        dataLoader_.loadData("/home/autodrive/Data/final/2_2_1_1/");
        std::cout << "Total No. of loaded data: " << dataLoader_.getDataSize() << std::endl;

        for (size_t i = 0 ; i < dataLoader_.getDataSize() ; i++) {
                auto data = dataLoader_.getNextData();
                std::cout << data->getTimestamp() << " " << data->toString() << std::endl;

                if (data->getType() == DataLoader::DataModelTypes::kLidarScanDataModelType) {
                        auto lidarData = std::dynamic_pointer_cast<DataLoader::LidarScanDataModel>(data);

                        auto scan = lidarData->getScan();

                }

                /* ... data processing ... */
        }
    }
}