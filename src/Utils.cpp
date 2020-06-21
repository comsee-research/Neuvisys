#include "Utils.hpp"
#include "Config.hpp"
#include <random>

namespace Util {
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

    xt::xarray<double> uniformMatrixPooling(const int row, const int col, const int layer) {
        unsigned seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);;

        xt::xarray<double> xray = xt::zeros<double>({layer, row, col});
        for (int k = 0; k < layer; ++k) {
            for (int i = 0; i < row; ++i) {
                for (int j = 0; j < col; ++j) {
                    xray(k, i, j) = distribution(generator);
                }
            }
        }
        return xray;
    }

    xt::xarray<double> uniformMatrixSynapses(const int row, const int col, const int nbSynapses) {
        unsigned seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        xt::xarray<double> xray = xt::zeros<double>({2, nbSynapses, row, col});
        for (int i = 0; i < row; ++i) {
            for (int j = 0; j < col; ++j) {
                for (int k = 0; k < 2; ++k) {
                    double weight = distribution(generator);
                    for (int l = 0; l < nbSynapses; ++l) {
                        xray(k, l, i, j) = weight;
                    }
                }
            }
        }
        return xray;
    }
}

Luts::Luts(double tauM, double tauRP, double tauSRA) : lutM(expLUT(tauM)), lutRP(expLUT(tauRP)), lutSRA(expLUT(tauSRA)) {}

std::vector<double> Luts::expLUT(double tau) {
    std::vector<double> exponential(1000000);
    for (int i = 0; i < 1000000; ++i) {
        exponential[i] = exp(- i / tau);
    }
    return exponential;
}