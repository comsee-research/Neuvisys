#ifndef NEUVISYS_DV_UTILS_HPP
#define NEUVISYS_DV_UTILS_HPP

#include <opencv2/core/mat.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"

namespace Util {
    xt::xarray<double> opencvMatToXarray(cv::Mat mat, int row, int col);
    xt::xarray<double> uniformMatrixPooling(int row, int col, int layer);
    xt::xarray<double> uniformMatrixSynapses(int row, int col, int nbSynapses);
}

class Luts {
public:
    Luts(double tauM, double tauRP, double tauSRA);
    std::vector<double> const lutM;
    std::vector<double> const lutRP;
    std::vector<double> const lutSRA;
private:
    static std::vector<double> expLUT(double tau);
};

#endif //NEUVISYS_DV_UTILS_HPP
