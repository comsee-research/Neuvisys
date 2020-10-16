#include "Utils.hpp"
#include "Config.hpp"
#include <random>

namespace Util {
    xt::xarray<double> uniformMatrixComplex(const size_t x, const size_t y, const size_t z) {
        unsigned seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        xt::xarray<double> xray = xt::zeros<double>({x, y, z});
        for (size_t i = 0; i < x; ++i) {
            for (size_t j = 0; j < y; ++j) {
                for (size_t k = 0; k < z; ++k) {
                    xray(i, j, k) = distribution(generator);
                }
            }
        }
        return xray;
    }

    xt::xarray<double> uniformMatrixSimple(const size_t x, const size_t y, const size_t nbSynapses) {
        unsigned seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        xt::xarray<double> xray = xt::zeros<double>({static_cast<size_t>(2), nbSynapses, x, y});
        for (size_t i = 0; i < x; ++i) {
            for (size_t j = 0; j < y; ++j) {
                for (size_t k = 0; k < 2; ++k) {
                    double weight = distribution(generator);
                    for (size_t l = 0; l < nbSynapses; ++l) {
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
    for (size_t i = 0; i < 1000000; ++i) {
        exponential[i] = exp(- static_cast<double>(i) / tau);
    }
    return exponential;
}