#include "Utils.hpp"

namespace Util {
    Eigen::Tensor<double, COMPLEXDIM> uniformMatrixComplex(const long x, const long y, const long z) {
        unsigned seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        Eigen::Tensor<double, COMPLEXDIM> mat(x, y, z);
        for (long i = 0; i < x; ++i) {
            for (long j = 0; j < y; ++j) {
                for (long k = 0; k < z; ++k) {
                    mat(i, j, k) = distribution(generator);
                }
            }
        }
        return mat;
    }

    Eigen::Tensor<double, SIMPLEDIM> uniformMatrixSimple(const long p, const long c, const long s, const long x, const long y) {
        unsigned seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        Eigen::Tensor<double, SIMPLEDIM> mat(p, c, s, x, y);
        for (long i = 0; i < x; ++i) {
            for (long j = 0; j < y; ++j) {
                for (long k = 0; k < p; ++k) {
                    double weight = distribution(generator);
                    for (long l = 0; l < c; ++l) {
                        for (long m = 0; m < s; ++m) {
                            mat(k, l, m, i, j) = weight;
                        }
                    }
                }
            }
        }
        return mat;
    }

    void loadNumpyFileToComplexTensor(std::string &filePath, Eigen::Tensor<double, COMPLEXDIM> &tensor) {
        cnpy::NpyArray array = cnpy::npy_load(filePath + ".npy");
        auto *weights = array.data<double>();

        const Eigen::Tensor<double, COMPLEXDIM>::Dimensions& d = tensor.dimensions();
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

    void saveComplexTensorToNumpyFile(Eigen::Tensor<double, COMPLEXDIM> tensor, std::string &saveFile) {
        const Eigen::Tensor<double, COMPLEXDIM>::Dimensions& d = tensor.dimensions();
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

    void loadNumpyFileToSimpleTensor(std::string &filePath, Eigen::Tensor<double, SIMPLEDIM> &tensor) {
        cnpy::NpyArray array = cnpy::npy_load(filePath + ".npy");
        auto *weights = array.data<double>();

        const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& d = tensor.dimensions();
        size_t count = 0;
        for (long i = 0; i < d[0]; ++i) {
            for (long j = 0; j < d[1]; ++j) {
                for (long k = 0; k < d[2]; ++k) {
                    for (long l = 0; l < d[3]; ++l) {
                        for (int m = 0; m < d[4]; ++m) {
                            tensor(i, j, k, l, m) = weights[count];
                            ++count;
                        }
                    }
                }
            }
        }
    }

    void saveSimpleTensorToNumpyFile(Eigen::Tensor<double, SIMPLEDIM> tensor, std::string &saveFile) {
        const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& d = tensor.dimensions();
        std::vector<double> data(static_cast<size_t>(d[0] * d[1] * d[2] * d[3] * d[4]));
        size_t count = 0;
        for (long i = 0; i < d[0]; ++i) {
            for (long j = 0; j < d[1]; ++j) {
                for (long k = 0; k < d[2]; ++k) {
                    for (long l = 0; l < d[3]; ++l) {
                        for (int m = 0; m < d[4]; ++m) {
                            data[count] = tensor(i, j, k, l, m);
                            ++count;
                        }
                    }
                }
            }
        }

        cnpy::npy_save(saveFile + ".npy", &data[0], {static_cast<size_t>(d[0]), static_cast<size_t>(d[1]), static_cast<size_t>(d[2]), static_cast<size_t>(d[3]), static_cast<size_t>(d[4])}, "w");
    }

    int winnerTakeAll(std::vector<size_t> v) {
        std::vector<size_t> argsmax;
        size_t max = v[0];

        for (size_t i = 0; i < v.size(); ++i) {
            if (v[i] > max) {
                max = v[i];
                argsmax.clear();
            }

            if (v[i] == max) {
                argsmax.push_back(i);
            }
        }

        if (argsmax.empty()) {
            return -1;
        } else {
            std::vector<int> randomArgmax;
            std::sample(argsmax.begin(), argsmax.end(), std::back_inserter(randomArgmax), 1, std::mt19937{std::random_device{}()});
            return randomArgmax[0];
        }
    }

    bool fileExist(std::string &path) {
        if (FILE *file = fopen(path.c_str(), "r")) {
            fclose(file);
            return true;
        } else {
            return false;
        }
    }
}

Luts::Luts(double tauM, double tauRP, double tauSRA) : lutM(expLUT(tauM)), lutRP(expLUT(tauRP)), lutSRA(expLUT(tauSRA)) {}

std::vector<double> Luts::expLUT(double tau) {
    if (tau > 0) {
        std::vector<double> exponential(1000000);
        for (size_t i = 0; i < 1000000; ++i) {
            exponential[i] = exp(- static_cast<double>(i) / tau);
        }
        return exponential;
    } else {
        throw std::runtime_error("Warning: tau parameter is not valid");
    }
}
