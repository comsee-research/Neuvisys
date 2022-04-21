#include "Util.hpp"

std::mt19937 generator(static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count()));
std::normal_distribution<double> normalDistr(0.0, 1.0);
std::uniform_real_distribution<double> uniformRealDistr(0.0, 1.0);

namespace Util {

    bool fileExist(std::string &filePath) {
        if (FILE *file = fopen(filePath.c_str(), "r")) {
            fclose(file);
            return true;
        } else {
            return false;
        }
    }

    bool endsWith(std::string const & value, std::string const & ending) {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    }

    Eigen::Tensor<double, COMPLEXDIM> uniformMatrixComplex(const long x, const long y, const long z) {
        Eigen::Tensor<double, COMPLEXDIM> mat(x, y, z);
        for (long i = 0; i < x; ++i) {
            for (long j = 0; j < y; ++j) {
                for (long k = 0; k < z; ++k) {
                    mat(i, j, k) = uniformRealDistr(generator);
                }
            }
        }
        return mat;
    }

    Eigen::Tensor<double, SIMPLEDIM> uniformMatrixSimple(const long p, const long c, const long s, const long x, const long y) {
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
        return mat;
    }

    void loadNumpyFileToSimpleTensor(Eigen::Tensor<double, SIMPLEDIM> &tensor, std::string &filePath, std::string &arrayName) {
        cnpy::NpyArray array;
        if (endsWith(filePath, ".npz")) {
            array = cnpy::npz_load(filePath, arrayName);
        } else if (endsWith(filePath, ".npy")) {
            array = cnpy::npy_load(filePath);
        }
        auto *weights = array.data<double>();

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

    void loadNumpyFileToComplexTensor(Eigen::Tensor<double, COMPLEXDIM> &tensor, std::string &filePath, std::string &arrayName) {
        cnpy::NpyArray array;
        if (endsWith(filePath, ".npz")) {
            array = cnpy::npz_load(filePath, arrayName);
        } else if (endsWith(filePath, ".npy")) {
            array = cnpy::npy_load(filePath);
        }
        auto *weights = array.data<double>();

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

    void loadNPYWeights(std::unordered_map<size_t, double> &map, std::string &filePath, std::string &arrayName) {
        cnpy::NpyArray array;
        if (endsWith(filePath, ".npz")) {
            array = cnpy::npz_load(filePath, arrayName);
        } else if (endsWith(filePath, ".npy")) {
            array = cnpy::npy_load(filePath);
        }
        auto *weights = array.data<double>();

        size_t count = 0;
        for (auto &element: map) {
            element.second = weights[count];
            ++count;
        }
    }

    void loadNumpyWeightsNPZ(std::unordered_map<size_t, double> &map, std::string &filePath, std::string &arrayName) {
        cnpy::NpyArray array = cnpy::npz_load(filePath, arrayName);
        auto *weights = array.data<double>();

        size_t count = 0;
        for (auto &element: map) {
            element.second = weights[count];
            ++count;
        }
    }

    void saveSimpleTensorToNPZ(Eigen::Tensor<double, SIMPLEDIM> tensor, std::string &filePath, std::string &arrayName) {
        const Eigen::Tensor<double, SIMPLEDIM>::Dimensions &d = tensor.dimensions();
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
        cnpy::npz_save(filePath, arrayName, &data[0],{static_cast<size_t>(d[0]), static_cast<size_t>(d[1]),
                                           static_cast<size_t>(d[2]), static_cast<size_t>(d[3]), static_cast<size_t>(d[4])}, "a");
    }

    void saveComplexTensorToNPZ(Eigen::Tensor<double, COMPLEXDIM> tensor, std::string &filePath, std::string &arrayName) {
        const Eigen::Tensor<double, COMPLEXDIM>::Dimensions &d = tensor.dimensions();
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

        cnpy::npz_save(filePath, arrayName, &data[0],
                       {static_cast<size_t>(d[0]), static_cast<size_t>(d[1]), static_cast<size_t>(d[2])}, "a");
    }

    void saveWeightsToNPZ(std::unordered_map<size_t, double> &map, std::string &filePath, std::string &arrayName) {
        std::vector<double> data(static_cast<size_t>(map.size()));
        size_t count = 0;
        for (auto const &element: map) {
            data[count] = element.second;
            ++count;
        }
        cnpy::npz_save(filePath, arrayName, &data[0], {map.size()}, "a");
    }

    void saveEventFile(std::vector<Event> &events, std::string &filePath) {
        std::vector<double> timestamp(events.size());
        std::vector<double> x(events.size());
        std::vector<double> y(events.size());
        std::vector<double> polarity(events.size());
        size_t count = 0;
        for (auto const &event: events) {
            timestamp[count] = static_cast<double>(event.timestamp());
            x[count] = event.x();
            y[count] = event.y();
            polarity[count] = event.polarity();
            ++count;
        }
        cnpy::npz_save(filePath + ".npz", "arr_0", &timestamp[0], {events.size()}, "w");
        cnpy::npz_save(filePath + ".npz", "arr_1", &x[0], {events.size()}, "a");
        cnpy::npz_save(filePath + ".npz", "arr_2", &y[0], {events.size()}, "a");
        cnpy::npz_save(filePath + ".npz", "arr_3", &polarity[0], {events.size()}, "a");
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

    double ornsteinUhlenbeckProcess(double &pos, double dt, double theta, double mu, double sigma) {
        double noise = normalDistr(generator) * std::sqrt(dt);
        pos = theta * (mu - pos) * dt + sigma * noise;
    }
}

Luts::Luts(double tauM, double tauRP, double tauSRA) : lutM(expLUT(tauM)), lutRP(expLUT(tauRP)), lutSRA(expLUT(tauSRA)) {}

std::vector<double> Luts::expLUT(double tau) {
    if (tau > 0) {
        std::vector<double> exponential(1000000);
        for (size_t i = 0; i < 1000000; ++i) {
            exponential[i] = exp(-static_cast<double>(i) / tau);
        }
        return exponential;
    } else {
        throw std::runtime_error("Warning: tau parameter is not valid");
    }
}
