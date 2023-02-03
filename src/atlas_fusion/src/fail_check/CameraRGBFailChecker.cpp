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
#include "util/polyfit.h"
#include "util/IdentifierToFrameConversions.h"

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
        if (data == nullptr || data->getImage().empty()) return;
        frameType_ = frameTypeFromDataModel(data);
        Timer t(frameTypeName(frameType_) + " validation");

        frameBgr = data->getImage();
        cv::cvtColor(frameBgr, frameGray, cv::COLOR_BGR2GRAY);

        std::vector<std::future<void>> futures;

        if (frameType_ == FrameType::kCameraLeftFront || frameType_ == FrameType::kCameraRightFront) {
            estimateVanishingPoint();
            futures.push_back(
                    context_.threadPool_.submit([&]() {
                        Timer t("Visibility at Vanishing point");
                        calculateVisibility(true, std::pair(vanishingPointX, vanishingPointY), std::pair(-1, -1));
                    }));
        }

        futures.push_back(context_.threadPool_.submit([&]() {
            Timer t("Glare and Occlusion");
            detectGlareAndOcclusion();
        }));

        for (int row = 0; row < DFT_BLOCK_COUNT; row++) {
            for (int col = 0; col < DFT_BLOCK_COUNT; col++) {
                int w = (DFT_WINDOW_SIZE / 2) + (col * ((frameGray.cols - DFT_WINDOW_SIZE) / (DFT_BLOCK_COUNT - 1)));
                int h = (DFT_WINDOW_SIZE / 2) + (row * ((frameGray.rows - DFT_WINDOW_SIZE) / (DFT_BLOCK_COUNT - 1)));
                futures.push_back(context_.threadPool_.submit(
                        [&, w, h, row, col]() { calculateVisibility(false, std::pair(w, h), std::pair(col, row)); }));
            }
        }

        for (auto &future: futures) {
            future.wait();
        }

        evaluatePerformance();
    }

    void
    CameraRGBFailChecker::onNewCameraDetections(
            const std::vector<std::pair<DataModels::FrustumDetection, std::set<FrameType>>> &detectionList) {
        detections = 0;
        detectionsVerified = 0;
        for (const auto &det: detectionList) {
            auto detFrames = det.second;
            // If the detection is seen by more than current camera, add to verified detections
            if (detFrames.count(frameType_) > 0) {
                detections++;
                if(detFrames.size() > 1) {
                    detectionsVerified++;
                }
            }
        }
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

    void CameraRGBFailChecker::calculateVisibility(bool isVanishingPoint, std::pair<int, int> centerPoint,
                                                   std::pair<int, int> position) {

        int32_t window_top_left_x = int(centerPoint.first) - (DFT_WINDOW_SIZE / 2);
        int32_t window_top_left_y = int(centerPoint.second) - (DFT_WINDOW_SIZE / 2);
        cv::Rect window_rect(window_top_left_x, window_top_left_y, DFT_WINDOW_SIZE, DFT_WINDOW_SIZE);

        cv::Mat cameraFrameWindow = frameGray(window_rect);

        int32_t optimalWindowSize = cv::getOptimalDFTSize(DFT_WINDOW_SIZE);
        cv::Mat paddedWindow;
        cv::copyMakeBorder(cameraFrameWindow, paddedWindow, 0, optimalWindowSize - cameraFrameWindow.rows, 0,
                           optimalWindowSize - cameraFrameWindow.cols,
                           cv::BORDER_CONSTANT, cv::Scalar::all(0));

        cv::Mat planes[] = {cv::Mat_<float>(paddedWindow), cv::Mat::zeros(paddedWindow.size(), CV_32F)};
        cv::Mat complexI;
        cv::merge(planes, 2, complexI);

        cv::dft(complexI, complexI);

        cv::split(complexI, planes);
        cv::magnitude(planes[0], planes[1], planes[0]);
        cv::Mat magI = planes[0];
        cv::Mat magIPow = magI.mul(magI);
        cv::Mat magINorm = magIPow.mul(DFT_WINDOW_SIZE ^ 2);

        magINorm += cv::Scalar::all(1);
        cv::log(magINorm, magINorm);

        // Crop the spectrum, if it has an odd number of rows or columns
        magINorm = magINorm(cv::Rect(0, 0, magI.cols & -2, magI.rows & -2));

        // Rearrange the quadrants of Fourier image so that the origin is at the image center
        int cx = magINorm.cols / 2;
        int cy = magINorm.rows / 2;
        cv::Mat q0(magINorm, cv::Rect(0, 0, cx, cy));
        cv::Mat q1(magINorm, cv::Rect(cx, 0, cx, cy));
        cv::Mat q2(magINorm, cv::Rect(0, cy, cx, cy));
        cv::Mat q3(magINorm, cv::Rect(cx, cy, cx, cy));
        cv::Mat tmp;
        q0.copyTo(tmp);
        q3.copyTo(q0);
        tmp.copyTo(q3);
        q1.copyTo(tmp);
        q2.copyTo(q1);
        tmp.copyTo(q2);

        cv::Mat magINormPolar;
        cv::warpPolar(magINorm, magINormPolar, cv::Size(magI.cols, magI.rows),
                      cv::Point2f(float(magI.cols) / 2, float(magI.rows) / 2), double(magI.rows) / 2.0,
                      cv::WARP_POLAR_LINEAR);
        cv::Mat intMagINormPolar = cv::Mat_<uint8_t>(magINormPolar);

        cv::normalize(magINorm, magINorm, 0, 255, cv::NORM_MINMAX);
        cv::Mat intMagINorm = cv::Mat_<uint8_t>(magINorm);

        std::vector<double> pss(DFT_WINDOW_SIZE);
        std::vector<double> freq(DFT_WINDOW_SIZE);
        for (int i = 0; i < intMagINormPolar.rows; i++) {
            for (int j = 0; j < intMagINormPolar.cols; j++) {
                pss[j] += intMagINormPolar.data[j + i * intMagINormPolar.cols];
                if (i == 0) freq[j] = (j + i * intMagINormPolar.cols) / 2.0;
            }
        }

        std::vector<double> coeffs(3);
        polyfit(freq, pss, coeffs, 2);

        if (isVanishingPoint) {
            vanishingPointVisibility = (1.0 - MOVING_AVERAGE_FORGET_RATE) * vanishingPointVisibility +
                                       MOVING_AVERAGE_FORGET_RATE * (1 - (MAX_VISIBILITY_THRESHOLD -
                                                                          std::min(MAX_VISIBILITY_THRESHOLD,
                                                                                   std::max(MIN_VISIBILITY_THRESHOLD,
                                                                                            coeffs[0]))) /
                                                                         (MAX_VISIBILITY_THRESHOLD -
                                                                          MIN_VISIBILITY_THRESHOLD));
        } else {
            double vis = (1.0 - (MAX_VISIBILITY_THRESHOLD -
                                 std::min(MAX_VISIBILITY_THRESHOLD, std::max(MIN_VISIBILITY_THRESHOLD, coeffs[0]))) /
                                (MAX_VISIBILITY_THRESHOLD - MIN_VISIBILITY_THRESHOLD));
            visibility.at<float>(position.first, position.second) = float(vis);
        }
    }

    void CameraRGBFailChecker::detectGlareAndOcclusion() {
        // Step 1: Convert the frame to a color space that maximizes resolution in luminance
        // Step 2: Divide the camera frame into meaningful regions to detect glare in
        int roiWidth = frameGray.cols / HISTOGRAM_COUNT;
        int roiHeight = frameGray.rows / HISTOGRAM_COUNT;
        int glareBinThreshold = int(HISTOGRAM_BINS * GLARE_THRESHOLD);
        int occlusionBinThreshold = int(HISTOGRAM_BINS * OCCLUSION_THRESHOLD);

        for (int i = 0, j = 0; i < HISTOGRAM_COUNT && j < HISTOGRAM_COUNT;) {
            int histIndex = i + (j * HISTOGRAM_COUNT);
            cv::Rect roi = cv::Rect(i * roiWidth, j * roiHeight, roiWidth, roiHeight);
            cv::Mat imageRoi = cv::Mat(frameGray, roi);

            // Step 3: Calculate histogram of every camera frame region
            cv::Mat histRoi;
            int histSize = HISTOGRAM_BINS;
            float range[] = {0, 256};
            const float *histRange[] = {range};
            cv::calcHist(&imageRoi, 1, nullptr, cv::Mat(), histRoi, 1, &histSize, histRange, true, false);
            histRoi = histRoi.t();

            for (int k = 0; k < histSize; k++) {
                histograms.at<float>(histIndex, k) = histRoi.at<float>(k);
            }

            // Step 4: Detect glare and occlusions in each histogram
            float occlusionLuminance = 0, regularLuminance = 0, glareLuminance = 0;
            for (int k = 0; k < HISTOGRAM_BINS; k++) {
                if (k >= glareBinThreshold) {
                    glareLuminance += histograms.at<float>(histIndex, k);
                } else if (k <= occlusionBinThreshold) {
                    occlusionLuminance += histograms.at<float>(histIndex, k);
                } else {
                    regularLuminance += histograms.at<float>(histIndex, k);
                }
            }

            if (glareLuminance >= (regularLuminance + occlusionLuminance)) {
                occlusionBuffers.at(histIndex).push_back(true);
                glareAmounts.at<float>(i, j) =
                        (glareLuminance * 10.0f) / (regularLuminance + glareLuminance + occlusionLuminance);
            } else if (occlusionLuminance >= (regularLuminance + glareLuminance)) {
                occlusionBuffers.at(histIndex).push_back(false);
                glareAmounts.at<float>(i, j) = occlusionBuffers.at(histIndex).dataSum() == 0L ? -1 : 0;
            } else {
                occlusionBuffers.at(histIndex).push_back(true);
                glareAmounts.at<float>(i, j) = 0;
            }

            // For daylight images ignore skylight
            if (j < HISTOGRAM_COUNT / 2 && environmentalModel_.getIsDaylight()) {
                if (glareAmounts.at<float>(i, j) > 0) {
                    glareAmounts.at<float>(i, j) = 0;
                }
            }

            i++;
            if (i == HISTOGRAM_COUNT) {
                i = 0;
                j++;
            }
        }
    }

    void CameraRGBFailChecker::evaluatePerformance() {
        float visibilitySum = 0.0f;
        for (int row = 0; row < DFT_BLOCK_COUNT; row++) {
            for (int col = 0; col < DFT_BLOCK_COUNT; col++) {
                float vis = visibility.at<float>(row, col);

                // First row
                if (row == 0) {
                    visibilitySum += vis;
                    continue;
                }

                // Rest of the rows have higher importance so their performance is penalized;
                visibilitySum += vis - 0.5f;
            }
        }

        float occludedCount = 0, glaredCount = 0;
        for (int row = 0; row < HISTOGRAM_COUNT; row++) {
            for (int col = 0; col < HISTOGRAM_COUNT; col++) {
                float glareAmount = glareAmounts.at<float>(row, col);
                float importance = 1.0f;
                if (row > 2 && col > 2 && row < HISTOGRAM_COUNT - 2 && col < HISTOGRAM_COUNT - 2) importance *= 2.0f;

                if (glareAmount == 0) continue;

                // Occlusion
                if (glareAmount < 0) occludedCount += importance;
                    // Glare
                else glaredCount += importance * glareAmount;
            }
        }

        nightShot = !environmentalModel_.getIsDaylight();
        possibleFog = visibilitySum < 0;
        possibleGlare = glaredCount > 10;
        possibleOcclusion = occludedCount > 10;

        sensorStatus_.statusVector.clear();
        sensorStatus_.statusString = frameTypeName(frameType_) + " status\n";
        sensorStatus_.statusVector.emplace_back(visibilitySum);
        sensorStatus_.statusString += "Vis sum: " + std::to_string(visibilitySum) + "\n";


        if (frameType_ == FrameType::kCameraLeftFront || frameType_ == FrameType::kCameraRightFront) {
            sensorStatus_.statusVector.emplace_back(vanishingPointVisibility);
            sensorStatus_.statusString += "VP vis: " + std::to_string(vanishingPointVisibility) + "\n";
        }

        sensorStatus_.statusVector.emplace_back(occludedCount);
        sensorStatus_.statusString += "Occluded count: " + std::to_string(occludedCount) + "\n";

        sensorStatus_.statusVector.emplace_back(glaredCount);
        sensorStatus_.statusString += "Glared count: " + std::to_string(glaredCount) + "\n";

        sensorStatus_.statusVector.emplace_back(possibleFog);
        sensorStatus_.statusString += "Possible fog: " + std::to_string(possibleFog) + "\n";

        sensorStatus_.statusVector.emplace_back(possibleGlare);
        sensorStatus_.statusString += "Possible glare: " + std::to_string(possibleGlare) + "\n";

        sensorStatus_.statusVector.emplace_back(possibleOcclusion);
        sensorStatus_.statusString += "Possible occlusion: " + std::to_string(possibleOcclusion) + "\n";

        sensorStatus_.statusVector.emplace_back(nightShot);
        sensorStatus_.statusString += "Night shot: " + std::to_string(nightShot) + "\n";

        sensorStatus_.statusVector.emplace_back(detections);
        sensorStatus_.statusVector.emplace_back(detectionsVerified);
        sensorStatus_.statusString +=
                "Verified/All detections: " + std::to_string(detectionsVerified) + "/" + std::to_string(detections) +
                "\n";
    }
}