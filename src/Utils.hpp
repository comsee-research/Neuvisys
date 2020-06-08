#ifndef NEUVISYS_DV_UTILS_HPP
#define NEUVISYS_DV_UTILS_HPP

#include <opencv2/core/mat.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"

namespace Util {
    inline std::vector<double> const& expLUT(double tau) {
        std::vector<double> temp;
        for (int i = 0; i < 1000000; ++i) {
            temp.push_back(exp(- i / tau));
        }
        static std::vector<double> const exponential(temp);
        return exponential;
    }

    xt::xarray<double> opencvMatToXarray(cv::Mat mat, int row, int col);
    xt::xarray<double> uniformMatrix(int row, int col);
    xt::xarray<double> uniformMatrixSynapses(int row, int col, int nbSynapses);
}

#endif //NEUVISYS_DV_UTILS_HPP
