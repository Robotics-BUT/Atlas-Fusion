#include "data_models/yolo/YoloDetectionClass.h"

namespace AutoDrive::DataModels {

    ReducedYoloDetectionClasses getReducedDetectionClass(YoloDetectionClass& cls) {

        auto detectionClass = ReducedYoloDetectionClasses::kOther;
        switch(cls) {
            case YoloDetectionClass::kPerson:
                detectionClass = ReducedYoloDetectionClasses::kPedestrian; break;
            case YoloDetectionClass::kBicycle:
            case YoloDetectionClass::kMotorbike:
                detectionClass = ReducedYoloDetectionClasses::kBike; break;
            case YoloDetectionClass::kCar:
            case YoloDetectionClass::kBus:
            case YoloDetectionClass::kTrain:
            case YoloDetectionClass::kTruck:
            case YoloDetectionClass::kBoat:
                detectionClass = ReducedYoloDetectionClasses::kVehicle; break;
            case YoloDetectionClass::kBird:
            case YoloDetectionClass::kCat:
            case YoloDetectionClass::kDog:
            case YoloDetectionClass::kHorse:
            case YoloDetectionClass::kCow:
            case YoloDetectionClass::kSheep:
            case YoloDetectionClass::kElephant:
            case YoloDetectionClass::kBear:
            case YoloDetectionClass::kZebra:
            case YoloDetectionClass::kGiraffe:
                detectionClass = ReducedYoloDetectionClasses::kAnimal; break;
            case YoloDetectionClass::kTrafficLight:
            case YoloDetectionClass::kStopSign:
                detectionClass = ReducedYoloDetectionClasses::kTraffic; break;
            default:
                break;
        }
        return detectionClass;
    }
}