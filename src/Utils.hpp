#ifndef NEUVISYS_DV_UTILS_HPP
#define NEUVISYS_DV_UTILS_HPP

#include <opencv2/core/mat.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"

xt::xarray<double> opencvMatToXarray(const cv::Mat mat, int row, int col);
xt::xarray<double> uniformMatrix(int row, int col);
xt::xarray<double> uniformMatrix2(int row, int col, int nbSynapses);

#endif //NEUVISYS_DV_UTILS_HPP
