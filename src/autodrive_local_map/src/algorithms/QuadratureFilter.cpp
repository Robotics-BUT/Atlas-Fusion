#include "algorithms/QuadratureFilter.h"
#include <cmath>
#include <iostream>

namespace AutoDrive::Algorithms {


    void QuadratureFilter::measurement(double x, double gain) {
        std::cout << "Measurement: " << std::endl;
        double sinX = sin(x);
        double cosX = cos(x);

        sinX_ = sinX_ * (1-gain) + sinX * (gain);
        cosX_ = cosX_ * (1-gain) + cosX * (gain);

        std::cout << "x: " << sinX_ << " " << cosX_ << std::endl;
    }


    void QuadratureFilter::prediction(double dx) {

        std::cout << "Prediction: " << std::endl;

        double val = getState() + dx;

        sinX_ = sin(val);
        cosX_ = cos(val);

        std::cout << "x: " << sinX_ << " " << cosX_ << std::endl;

    }


    double QuadratureFilter::getState() const {
        return std::atan2(sinX_, cosX_);
    }


    double QuadratureFilter::setState(double x) {
        sinX_ = sin(x);
        cosX_ = cos(x);
    }
}