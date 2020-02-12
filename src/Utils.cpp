#include "Utils.hpp"
#include <stdlib.h>
#include <time.h>

xt::xarray<double> opencvMatToXarray(const cv::Mat mat, const int row, const int col) {
    xt::xarray<double> xray = xt::zeros<double>({2, row, col});
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            xray(0, i, j) = mat.at<double>(i, j);
            xray(1, i, j) = mat.at<double>(i, j);
        }
    }
    return xray;
}

xt::xarray<double> uniformMatrix(const int row, const int col) {
    srand(time(NULL));

    xt::xarray<double> xray = xt::zeros<double>({2, row, col});
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            for (int k = 0; k < 2; ++k) {
                xray(k, i, j) = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            }
        }
    }
    return xray;
}
