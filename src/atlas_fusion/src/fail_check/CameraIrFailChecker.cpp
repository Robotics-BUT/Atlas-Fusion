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

#include "fail_check/CameraIrFailChecker.h"
#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::FailCheck {

    void CameraIrFailChecker::onNewData(const std::shared_ptr<DataModels::CameraIrFrameDataModel>& data) {
        minTemp = data->getTemp().first;
        maxTemp = data->getTemp().second;

        // Road temp measurement
        auto roi = data->getImage();
        roi = roi(cv::Rect(280, 430, 60, 30));
        meanRoadROITemp = 0.995 * meanRoadROITemp + 0.005 * ((maxTemp - minTemp) * (cv::mean(roi).val[0] / 255.0f) + minTemp);
        environmentalModel_.onTemperature(meanRoadROITemp);

        sensorStatus_.statusVector.clear();
        sensorStatus_.statusString = frameTypeName(frameTypeFromIdentifier(data->getCameraIdentifier())) + " status\n";
        sensorStatus_.statusVector.emplace_back(minTemp);
        sensorStatus_.statusString += "Min temp: " + std::to_string(minTemp) + "\n";
        sensorStatus_.statusVector.emplace_back(maxTemp);
        sensorStatus_.statusString += "Max temp: " + std::to_string(maxTemp) + "\n";
        sensorStatus_.statusVector.emplace_back(maxTemp - minTemp);
        sensorStatus_.statusString += "Temp range: " + std::to_string(maxTemp - minTemp) + "\n";
        sensorStatus_.statusVector.emplace_back(meanRoadROITemp);
        sensorStatus_.statusString += "Road ROI temp: " + std::to_string(meanRoadROITemp) + "\n";
    }
}