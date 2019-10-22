#include "local_map/Yolo/YoloDetection.h"

namespace AutoDrive {
    namespace LocalMap {

        ReducedYoloDetectionClasses YoloDetection::getReducedDetectionClass() {

            auto detectionClass = ReducedYoloDetectionClasses::kOther;
            switch(detClass_) {
                case YoloDetectionClasses::kPerson:
                    detectionClass = ReducedYoloDetectionClasses::kPedestrian; break;
                case YoloDetectionClasses::kBicycle:
                case YoloDetectionClasses::kMotorbike:
                    detectionClass = ReducedYoloDetectionClasses::kBike; break;
                case YoloDetectionClasses::kCar:
                case YoloDetectionClasses::kBus:
                case YoloDetectionClasses::kTrain:
                case YoloDetectionClasses::kTruck:
                case YoloDetectionClasses::kBoat:
                    detectionClass = ReducedYoloDetectionClasses::kVehicle; break;
                case YoloDetectionClasses::kBird:
                case YoloDetectionClasses::kCat:
                case YoloDetectionClasses::kDog:
                case YoloDetectionClasses::kHorse:
                case YoloDetectionClasses::kSheep:
                case YoloDetectionClasses::kElephant:
                case YoloDetectionClasses::kBear:
                case YoloDetectionClasses::kZebra:
                case YoloDetectionClasses::kGiraffe:
                    detectionClass = ReducedYoloDetectionClasses::kAnimal; break;
            }
            return detectionClass;
        }
    }
}