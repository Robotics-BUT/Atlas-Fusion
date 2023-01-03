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
#include "Timer.h"

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
        Timer t("CameraRGBFailChecker");

        frameBgr = data->getImage();
        cv::cvtColor(frameBgr, frameGray, cv::COLOR_BGR2GRAY);
        estimateVanishingPoint();

        context_.threadPool_.push_task([&]() mutable { calculateVisibility(true, std::pair(vanishingPointX, vanishingPointY), std::pair(-1, -1)); });
        context_.threadPool_.push_task([&]() mutable { detectGlareAndOcclusion(); });

        for (int j = 0; j < DFT_BLOCK_COUNT; j++) {
            for (int i = 0; i < DFT_BLOCK_COUNT; i++) {
                int w = (DFT_WINDOW_SIZE / 2) + (i * ((frameGray.cols - DFT_WINDOW_SIZE) / (DFT_BLOCK_COUNT - 1)));
                int h = (DFT_WINDOW_SIZE / 2) + (j * ((frameGray.rows - DFT_WINDOW_SIZE) / (DFT_BLOCK_COUNT - 1)));
                context_.threadPool_.push_task([&]() mutable { calculateVisibility(false, std::pair(w, h), std::pair(i, j)); });
            }
        }
        context_.threadPool_.wait_for_tasks();
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

    void CameraRGBFailChecker::calculateVisibility(bool isVanishingPoint, std::pair<int, int> centerPoint, std::pair<int, int> position) {
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
                glareAmounts.at<float>(i, j) = (glareLuminance * 255.0f) / (regularLuminance + glareLuminance + occlusionLuminance);
            } else if (occlusionLuminance >= (regularLuminance + glareLuminance)) {
                occlusionBuffers.at(histIndex).push_back(false);
                glareAmounts.at<float>(i, j) = occlusionBuffers.at(histIndex).dataSum() == 0L ? 0 : 64;
            } else {
                occlusionBuffers.at(histIndex).push_back(true);
                glareAmounts.at<float>(i, j) = 64;
            }

            // For daylight images ignore skylight
            if (j < HISTOGRAM_COUNT / 2 && environmentalModel_.getIsDaylight()) {
                if (glareAmounts.at<float>(i, j) > 64) {
                    glareAmounts.at<float>(i, j) = 64;
                }
            }

            i++;
            if (i == HISTOGRAM_COUNT) {
                i = 0;
                j++;
            }
        }

        // Step 5: Calculate a global score for the image based on the detected glares in various regions
    }
}