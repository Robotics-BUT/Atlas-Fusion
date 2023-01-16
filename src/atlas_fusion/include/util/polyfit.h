#include "../../libs/eigen/Dense"
#include <cmath>
#include <vector>
#include "../../libs/eigen/QR"

void polyfit(const std::vector<double> &t, const std::vector<double> &v, std::vector<double> &coeff, int order) {
    // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
    Eigen::MatrixXd T(t.size(), order + 1);
    Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
    Eigen::VectorXd result;

    // check to make sure inputs are correct
    assert(t.size() == v.size());
    assert(t.size() >= order + 1);
    // Populate the matrix
    for (size_t i = 0; i < t.size(); ++i) {
        for (size_t j = 0; j < order + 1; ++j) {
            T(i, j) = pow(t.at(i), j);
        }
    }

    // Solve for linear least square fit
    result = T.householderQr().solve(V);
    coeff.resize(order + 1);
    for (int k = 0; k < order + 1; k++) {
        coeff[k] = result[k];
    }

}