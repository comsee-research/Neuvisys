#include "Utils.hpp"
#include "stdlib.h"

xt::xarray<double> opencvMatToXarray(const cv::Mat mat, int row, int col) {
    xt::xarray<double> xray = xt::zeros<double>({row, col});
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            xray(i, j) = mat.at<double>(i, j);
        }
    }
    return xray;
}

xt::xarray<double> uniformMatrix(int row, int col) {
    xt::xarray<double> xray = xt::zeros<double>({row, col});
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            xray(i, j) = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        }
    }
    return xray;
}