#include "Utils.hpp"

namespace Util {
    Eigen::Tensor<double, 3> uniformMatrixComplex(const long x, const long y, const long z) {
        unsigned seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        Eigen::Tensor<double, 3> mat(x, y, z);
        for (long i = 0; i < x; ++i) {
            for (long j = 0; j < y; ++j) {
                for (long k = 0; k < z; ++k) {
                    mat(i, j, k) = distribution(generator);
                }
            }
        }
        return mat;
    }

    Eigen::Tensor<double, 4> uniformMatrixSimple(const long s, const long x, const long y) {
        unsigned seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        Eigen::Tensor<double, 4> mat(2, s, x, y);
        for (long i = 0; i < x; ++i) {
            for (long j = 0; j < y; ++j) {
                for (long k = 0; k < 2; ++k) {
                    double weight = distribution(generator);
                    for (long l = 0; l < s; ++l) {
                        mat(k, l, i, j) = weight;
                    }
                }
            }
        }
        return mat;
    }

    void loadNumpyFileTo3DTensor(std::string &filePath, Eigen::Tensor<double, 3> &tensor) {
        cnpy::NpyArray array = cnpy::npy_load(filePath + ".npy");
        auto *weights = array.data<double>();

        const Eigen::Tensor<double, 3>::Dimensions& d = tensor.dimensions();
        size_t count = 0;
        for (long i = 0; i < d[0]; ++i) {
            for (long j = 0; j < d[1]; ++j) {
                for (long k = 0; k < d[2]; ++k) {
                    tensor(i, j, k) = weights[count];
                    ++count;
                }
            }
        }
    }

    void save3DTensorToNumpyFile(Eigen::Tensor<double, 3> tensor, std::string &saveFile) {
        const Eigen::Tensor<double, 3>::Dimensions& d = tensor.dimensions();
        std::vector<double> data(static_cast<size_t>(d[0] * d[1] * d[2]));
        size_t count = 0;
        for (long i = 0; i < d[0]; ++i) {
            for (long j = 0; j < d[1]; ++j) {
                for (long k = 0; k < d[2]; ++k) {
                    data[count] = tensor(i, j, k);
                    ++count;
                }
            }
        }

        cnpy::npy_save(saveFile + ".npy", &data[0], {static_cast<size_t>(d[0]), static_cast<size_t>(d[1]), static_cast<size_t>(d[2])}, "w");
    }

    void loadNumpyFileTo4DTensor(std::string &filePath, Eigen::Tensor<double, 4> &tensor) {
        cnpy::NpyArray array = cnpy::npy_load(filePath + ".npy");
        auto *weights = array.data<double>();

        const Eigen::Tensor<double, 4>::Dimensions& d = tensor.dimensions();
        size_t count = 0;
        for (long i = 0; i < d[0]; ++i) {
            for (long j = 0; j < d[1]; ++j) {
                for (long k = 0; k < d[2]; ++k) {
                    for (long l = 0; l < d[3]; ++l) {
                        tensor(i, j, k, l) = weights[count];
                        ++count;
                    }
                }
            }
        }
    }

    void save4DTensorToNumpyFile(Eigen::Tensor<double, 4> tensor, std::string &saveFile) {
        const Eigen::Tensor<double, 4>::Dimensions& d = tensor.dimensions();
        std::vector<double> data(static_cast<size_t>(d[0] * d[1] * d[2] * d[3]));
        size_t count = 0;
        for (long i = 0; i < d[0]; ++i) {
            for (long j = 0; j < d[1]; ++j) {
                for (long k = 0; k < d[2]; ++k) {
                    for (long l = 0; l < d[3]; ++l) {
                        data[count] = tensor(i, j, k, l);
                        ++count;
                    }
                }
            }
        }

        cnpy::npy_save(saveFile + ".npy", &data[0], {static_cast<size_t>(d[0]), static_cast<size_t>(d[1]), static_cast<size_t>(d[2]), static_cast<size_t>(d[3])}, "w");
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