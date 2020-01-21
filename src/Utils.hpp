#ifndef NEUVISYS_DV_UTILS_HPP
#define NEUVISYS_DV_UTILS_HPP

#endif //NEUVISYS_DV_UTILS_HPP

#include <opencv2/core/mat.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"

xt::xarray<double> opencvMatToXarray(const cv::Mat mat, int row, int col);