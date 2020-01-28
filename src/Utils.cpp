#include "Utils.hpp"
#include "stdlib.h"

xt::xarray<double> opencvMatToXarray(const cv::Mat mat, const int row, const int col) {
    xt::xarray<double> xray = xt::zeros<double>({row, col});
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            xray(i, j) = mat.at<double>(i, j);
        }
    }
    return xray;
}

xt::xarray<double> uniformMatrix(const int row, const int col) {
    xt::xarray<double> xray = xt::zeros<double>({row, col});
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            xray(i, j) = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        }
    }
    return xray;
}

void normalizeMatrix(xt::xarray<double> &xray) {
    double norm = xt::linalg::norm(xray);
    if (norm != 0) {
        xray = (xray / norm) * 64;
    }
}
