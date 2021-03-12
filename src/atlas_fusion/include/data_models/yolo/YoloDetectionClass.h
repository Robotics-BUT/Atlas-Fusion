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

namespace AtlasFusion::DataModels {

    /**
     * Full list of COCO dataset labels used for the video-yolo inference.
     */
    enum class YoloDetectionClass {
        kPerson = 0,
        kBicycle = 1,
        kCar = 2,
        kMotorbike = 3,
        kAeroplane = 4,
        kBus = 5,
        kTrain = 6,
        kTruck = 7,
        kBoat = 8,
        kTrafficLight = 9,
        kFireHydrant = 10,
        kStopSign = 11,
        kParkingMeter = 12,
        kBench = 13,
        kBird = 14,
        kCat = 15,
        kDog = 16,
        kHorse = 17,
        kSheep = 18,
        kCow = 19,
        kElephant = 20,
        kBear = 21,
        kZebra = 22,
        kGiraffe = 23,
        kBackpack = 24,
        kUmbrella = 25,
        kHandbag = 26,
        kTie = 27,
        kSuitcase = 28,
        kFrisbee = 29,
        kSkis = 30,
        kSnowboard = 31,
        kSportsBall = 32,
        kKite = 33,
        kBaseballBat = 34,
        kBaseballGlove = 35,
        kSkateboard = 36,
        kSurfboard = 37,
        kTennisRacket = 38,
        kBottle = 39,
        kWineGlass = 40,
        kCup = 41,
        kFork = 42,
        kKnife = 43,
        kSpoon = 44,
        kBowl = 45,
        kBanana = 46,
        kApple = 47,
        kSandwich = 48,
        kOrange = 49,
        kBroccoli = 50,
        kCarrot = 51,
        kHotDog = 52,
        kPizza = 53,
        kDonut = 54,
        kCake = 55,
        kChair = 56,
        kSofa = 57,
        kPottedplant = 58,
        kBed = 59,
        kDiningtable = 60,
        kToilet = 61,
        kTvmonitor = 62,
        kLaptop = 63,
        kMouse = 64,
        kRemote = 65,
        kKeyboard = 66,
        kCellPhone = 67,
        kMicrowave = 68,
        kOven = 69,
        kToaster = 70,
        kSink = 71,
        kRefrigerator = 72,
        kBook = 73,
        kClock = 74,
        kVase = 75,
        kScissors = 76,
        kTeddyBear = 77,
        kHairDrier = 78,
        kToothbrush = 79,
    };

    /**
     * Simplified YOLO detection classes. Reduces the number of classes which are used in this mapping system.
     */
    enum class ReducedYoloDetectionClasses {
        kPedestrian = 0,
        kBike = 1,
        kVehicle = 2,
        kAnimal = 3,
        kTraffic = 4,
        kOther = 5,
    };

    ReducedYoloDetectionClasses getReducedDetectionClass(YoloDetectionClass& cls);
}