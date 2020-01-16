#pragma once

namespace AutoDrive::Algorithms {

    class QuadratureFilter {

    public:

        QuadratureFilter() {

        }

        void measurement(double x, double gain);
        void prediction(double dx);

        double getState() const;
        double setState(double x);

    private:

        double sinX_ = 0;
        double cosX_ = 0;
    };
}

