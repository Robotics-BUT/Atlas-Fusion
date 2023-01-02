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

#include "fail_check/CameraRGBFailChecker.h"

namespace AutoDrive::FailCheck {

    void print(const std::string &name, double hor) {
        double d = hor;

        double pos = 0;
        std::cout << name << ": ";
        do {
            std::cout << ".";
            pos++;
        } while (hor-- > 0);

        std::cout << "*";

        while (pos <= 100) {
            std::cout << ".";
            pos++;
        }
        std::cout << " " << d << std::endl;
    }

    void CameraRGBFailChecker::onNewData(const std::shared_ptr<DataModels::CameraFrameDataModel> &data) {
        frameBgr = data->getImage();
        cv::cvtColor(frameBgr, frameGray, cv::COLOR_BGR2GRAY);
        estimateVanishingPoint();
    }

    void CameraRGBFailChecker::estimateVanishingPoint() {
        // Horizontal position (Heading difference adjusted to 0-100 range linearly with custom sensitivity)
        double horizontal = std::min(2.0, std::max((selfModel_.getHeadingDiff() * -700) + 1, 0.0)) * 50;
        //print("Horizontal position", horizontal);
        vanishingPointX = (frameGray.cols / 100.0) * horizontal;

        // Vertical position (Attitude difference adjusted to 0-100 range linearly with custom sensitivity)
        double vertical = std::min(2.0, std::max((selfModel_.getAttitudeDiff() * -700) + 1, 0.0)) * 50 - 5;
        //print("Vertical position", vertical);
        vanishingPointY = (frameGray.rows / 100.0) * vertical;

        //std::cout << "Vanishing point at: (" << vanishingPointX << ", " << vanishingPointY << ")" << std::endl;
    }

}