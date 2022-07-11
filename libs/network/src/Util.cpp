//
// Created by Thomas on 14/04/2021.
//

#include "Util.hpp"

//std
#include <random>
#include <chrono>

static std::mt19937 generator(static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count()));
static std::normal_distribution<double> normalDistr(0.0, 1.0);
static std::uniform_real_distribution<double> uniformRealDistr(0.0, 1.0);

void weightsToSimpleTensor(Eigen::Tensor<double, SIMPLEDIM> &tensor, const double *weights) {
    const Eigen::Tensor<double, SIMPLEDIM>::Dimensions &d = tensor.dimensions();
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

void weightsToComplexTensor(Eigen::Tensor<double, COMPLEXDIM> &tensor, const double *weights) {
    const Eigen::Tensor<double, COMPLEXDIM>::Dimensions &d = tensor.dimensions();
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

void weightsToMap(std::unordered_map<size_t, double> &map, const double *weights) {
    size_t count = 0;
    for (auto &element: map) {
        element.second = weights[count];
        ++count;
    }
}

void simpleTensorToWeights(const Eigen::Tensor<double, SIMPLEDIM> &tensor, std::vector<double> &data,
                           const Eigen::Tensor<double, SIMPLEDIM>::Dimensions &d) {
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
}

void complexTensorToWeights(const Eigen::Tensor<double, COMPLEXDIM> &tensor, std::vector<double> &data,
                            const Eigen::Tensor<double, COMPLEXDIM>::Dimensions &d) {
    size_t count = 0;
    for (long i = 0; i < d[0]; ++i) {
        for (long j = 0; j < d[1]; ++j) {
            for (long k = 0; k < d[2]; ++k) {
                data[count] = tensor(i, j, k);
                ++count;
            }
        }
    }
}

void mapToWeights(const std::unordered_map<size_t, double> &map, std::vector<double> &data) {
    size_t count = 0;
    for (auto const &element: map) {
        data[count] = element.second;
        ++count;
    }
}

namespace Util {
    bool fileExist(std::string &filePath) {
        if (FILE *file = fopen(filePath.c_str(), "r")) {
            fclose(file);
            return true;
        } else {
            return false;
        }
    }

    bool endsWith(std::string const &value, std::string const &ending) {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    }

    Eigen::Tensor<double, COMPLEXDIM> uniformMatrixComplex(const long x, const long y, const long z, const double normalizationFactor) {
        Eigen::Tensor<double, COMPLEXDIM> mat(x, y, z);
        for (long i = 0; i < x; ++i) {
            for (long j = 0; j < y; ++j) {
                for (long k = 0; k < z; ++k) {
                    mat(i, j, k) = uniformRealDistr(generator);
                }
            }
        }
        normalizeComplexTensor(mat, normalizationFactor);
        return mat;
    }

    Eigen::Tensor<double, SIMPLEDIM> uniformMatrixSimple(const long p, const long c, const long s, const long x, const long y, const double normalizationFactor) {
        Eigen::Tensor<double, SIMPLEDIM> mat(p, c, s, x, y);
        for (long i = 0; i < x; ++i) {
            for (long j = 0; j < y; ++j) {
                for (long k = 0; k < p; ++k) {
                    double weight = uniformRealDistr(generator);
                    for (long l = 0; l < c; ++l) {
                        for (long m = 0; m < s; ++m) {
                            mat(k, l, m, i, j) = weight;
                        }
                    }
                }
            }
        }
        normalizeSimpleTensor(mat, normalizationFactor);
        return mat;
    }

    void normalizeSimpleTensor(Eigen::Tensor<double, SIMPLEDIM> &weights, double normalizationFactor) {
        const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& d = weights.dimensions();

//    for (long p = 0; p < d[0]; ++p) {
//        for (int c = 0; c < d[1]; ++c) {
//            for (long s = 0; s < d[2]; ++s) {
//                Eigen::array<long, SIMPLEDIM> start = {p, c, s, 0, 0};
//                Eigen::array<long, SIMPLEDIM> size = {1, 1, 1, d[3], d[4]};
//                Eigen::Tensor<double, 0> norm = m_weights.slice(start, size).pow(2).sum().sqrt();
//
//                if (norm(0) != 0) {
//                    m_weights.slice(start, size) = m_conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
//                }
//            }
//        }
//    }

        // weight normalization on the camera axes
//    for (long c = 0; c < d[1]; ++c) {
//        Eigen::array<long, SIMPLEDIM> start = {0, c, 0, 0, 0};
//        Eigen::array<long, SIMPLEDIM> size = {d[0], 1, d[2], d[3], d[4]};
//        Eigen::Tensor<double, 0> norm = m_weights.slice(start, size).pow(2).sum().sqrt();
//
//        if (norm(0) != 0) {
//            m_weights.slice(start, size) = m_conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
//        }
//    }

        Eigen::Tensor<double, 0> norm = weights.pow(2).sum().sqrt();

        if (norm(0) != 0) {
            weights = normalizationFactor * weights / norm(0);
        }
    }

    void normalizeComplexTensor(Eigen::Tensor<double, COMPLEXDIM> &weights, double normalizationFactor) {
        Eigen::Tensor<double, 0> norm = weights.pow(2).sum().sqrt();

        if (norm(0) != 0) {
            weights = normalizationFactor * weights / norm(0);
        }
    }

    void loadNumpyFileToSimpleTensor(Eigen::Tensor<double, SIMPLEDIM> &tensor, std::string &filePath) {
        auto *weights = cnpy::npy_load(filePath).data<double>();
        weightsToSimpleTensor(tensor, weights);
    }

    void loadNumpyFileToSimpleTensor(Eigen::Tensor<double, SIMPLEDIM> &tensor, cnpy::npz_t &arrayNPZ, std::string &arrayName) {
        auto *weights = arrayNPZ[arrayName].data<double>();
        weightsToSimpleTensor(tensor, weights);
    }

    void loadNumpyFileToComplexTensor(Eigen::Tensor<double, COMPLEXDIM> &tensor, std::string &filePath) {
        auto *weights = cnpy::npy_load(filePath).data<double>();
        weightsToComplexTensor(tensor, weights);
    }

    void loadNumpyFileToComplexTensor(Eigen::Tensor<double, COMPLEXDIM> &tensor, cnpy::npz_t &arrayNPZ, std::string &arrayName) {
        auto *weights = arrayNPZ[arrayName].data<double>();
        weightsToComplexTensor(tensor, weights);
    }

    void loadNumpyFileToWeights(std::unordered_map<size_t, double> &map, std::string &filePath) {
        auto *weights = cnpy::npy_load(filePath).data<double>();
        weightsToMap(map, weights);
    }

    void loadNumpyFileToWeights(std::unordered_map<size_t, double> &map, cnpy::npz_t &arrayNPZ, std::string &arrayName) {
        auto *weights = arrayNPZ[arrayName].data<double>();
        weightsToMap(map, weights);
    }

    void saveSimpleTensorToNumpyFile(const Eigen::Tensor<double, SIMPLEDIM> &tensor, std::string &filePath) {
        const Eigen::Tensor<double, SIMPLEDIM>::Dimensions &d = tensor.dimensions();
        std::vector<double> data(static_cast<size_t>(d[0] * d[1] * d[2] * d[3] * d[4]));
        simpleTensorToWeights(tensor, data, d);
        cnpy::npy_save(filePath + ".npy", &data[0], {static_cast<size_t>(d[0]), static_cast<size_t>(d[1]),
                                                     static_cast<size_t>(d[2]), static_cast<size_t>(d[3]), static_cast<size_t>(d[4])}, "w");
    }

    void saveSimpleTensorToNumpyFile(const Eigen::Tensor<double, SIMPLEDIM> &tensor, std::string &filePath, std::string &arrayName) {
        const Eigen::Tensor<double, SIMPLEDIM>::Dimensions &d = tensor.dimensions();
        std::vector<double> data(static_cast<size_t>(d[0] * d[1] * d[2] * d[3] * d[4]));
        simpleTensorToWeights(tensor, data, d);
        cnpy::npz_save(filePath, arrayName, &data[0], {static_cast<size_t>(d[0]), static_cast<size_t>(d[1]),
                                                       static_cast<size_t>(d[2]), static_cast<size_t>(d[3]), static_cast<size_t>(d[4])}, "a");
    }

    void saveComplexTensorToNumpyFile(const Eigen::Tensor<double, COMPLEXDIM> &tensor, std::string &filePath) {
        const Eigen::Tensor<double, COMPLEXDIM>::Dimensions &d = tensor.dimensions();
        std::vector<double> data(static_cast<size_t>(d[0] * d[1] * d[2]));
        complexTensorToWeights(tensor, data, d);
        cnpy::npy_save(filePath + ".npy", &data[0], {static_cast<size_t>(d[0]), static_cast<size_t>(d[1]), static_cast<size_t>(d[2])}, "w");
    }

    void saveComplexTensorToNumpyFile(const Eigen::Tensor<double, COMPLEXDIM> &tensor, std::string &filePath, std::string &arrayName) {
        const Eigen::Tensor<double, COMPLEXDIM>::Dimensions &d = tensor.dimensions();
        std::vector<double> data(static_cast<size_t>(d[0] * d[1] * d[2]));
        complexTensorToWeights(tensor, data, d);
        cnpy::npz_save(filePath, arrayName, &data[0],
                       {static_cast<size_t>(d[0]), static_cast<size_t>(d[1]), static_cast<size_t>(d[2])}, "a");
    }

    void saveWeightsToNumpyFile(const std::unordered_map<size_t, double> &map, std::string &filePath) {
        std::vector<double> data(static_cast<size_t>(map.size()));
        mapToWeights(map, data);
        cnpy::npy_save(filePath + ".npy", &data[0], {map.size()}, "w");
    }

    void saveWeightsToNumpyFile(const std::unordered_map<size_t, double> &map, std::string &filePath, std::string &arrayName) {
        std::vector<double> data(static_cast<size_t>(map.size()));
        mapToWeights(map, data);
        cnpy::npz_save(filePath, arrayName, &data[0], {map.size()}, "a");
    }

    void saveEventFile(std::vector<Event> &events, std::string &filePath) {
        std::vector<double> timestamp(events.size());
        std::vector<double> x(events.size());
        std::vector<double> y(events.size());
        std::vector<double> polarity(events.size());
        std::vector<double> camera(events.size());
        size_t count = 0;
        for (auto const &event: events) {
            timestamp[count] = static_cast<double>(event.timestamp());
            x[count] = event.x();
            y[count] = event.y();
            polarity[count] = event.polarity();
            camera[count] = event.camera();
            ++count;
        }
        cnpy::npz_save(filePath + ".npz", "arr_0", &timestamp[0], {events.size()}, "w");
        cnpy::npz_save(filePath + ".npz", "arr_1", &x[0], {events.size()}, "a");
        cnpy::npz_save(filePath + ".npz", "arr_2", &y[0], {events.size()}, "a");
        cnpy::npz_save(filePath + ".npz", "arr_3", &polarity[0], {events.size()}, "a");
        cnpy::npz_save(filePath + ".npz", "arr_4", &camera[0], {events.size()}, "a");
    }

    int winnerTakeAll(std::vector<size_t> vec) {
        std::vector<size_t> argsmax;
        size_t max = 0;

        for (size_t i = 0; i < vec.size(); ++i) {
            if (vec[i] > max) {
                max = vec[i];
                argsmax.clear();
            }

            if (max != 0 && vec[i] == max) {
                argsmax.push_back(i);
            }
        }

        if (argsmax.empty()) {
            return -1;
        } else {
            std::vector<int> randomArgmax;
            std::sample(argsmax.begin(), argsmax.end(), std::back_inserter(randomArgmax), 1, generator);
            return randomArgmax[0];
        }
    }

    double secondOrderNumericalDifferentiationMean(const std::vector<double> &vec, long n) {
        if (!vec.empty()) {
            double sumDVec = 0;
            int count = 0;
            for (size_t i = vec.size() - 2; i > vec.size() - n && i > 0; --i) {
                sumDVec += (vec[i + 1] - vec[i - 1]) / 2.0;
                ++count;
            }
            return sumDVec / count;
        } else {
            return 0;
        }
    }

    void ornsteinUhlenbeckProcess(double &pos, double dt, double theta, double mu, double sigma) {
        double noise = normalDistr(generator) * std::sqrt(dt);
        pos = theta * (mu - pos) * dt + sigma * noise;
    }
}

//Luts::Luts(double tauM, double tauRP, double tauSRA) : lutM(expLUT(tauM)), lutRP(expLUT(tauRP)), lutSRA(expLUT(tauSRA)) {}
//
//std::vector<double> Luts::expLUT(double tau) {
//    if (tau > 0) {
//        std::vector<double> exponential(1000000);
//        for (size_t i = 0; i < 1000000; ++i) {
//            exponential[i] = exp(-static_cast<double>(i) / tau);
//        }
//        return exponential;
//    } else {
//        throw std::runtime_error("Warning: tau parameter is not valid");
//    }
//}
