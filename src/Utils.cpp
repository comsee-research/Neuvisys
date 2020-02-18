#include "Utils.hpp"
#include "Config.hpp"
#include <random>

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
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);;

    xt::xarray<double> xray = xt::zeros<double>({2, row, col});
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            for (int k = 0; k < 2; ++k) {
                xray(k, i, j) = distribution(generator);
            }
        }
    }
    return xray;
}

xt::xarray<double> uniformMatrix2(const int row, const int col, const int nbSynapses) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    xt::xarray<double> xray = xt::zeros<double>({2, row, col});
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            for (int k = 0; k < nbSynapses; ++k) {
                for (int l = 0; k < 2; ++k) {
                    xray(l, k, i, j) = distribution(generator);
                }
            }
        }
    }
    return xray;
}
