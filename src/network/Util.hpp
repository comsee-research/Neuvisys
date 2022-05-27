//
// Created by Thomas on 14/04/2021.
//

#ifndef NEUVISYS_DV_UTIL_HPP
#define NEUVISYS_DV_UTIL_HPP

#include "../dependencies/unsupported/Eigen/CXX11/Tensor"
#include <random>
#include <chrono>
#include "cnpy.h"
#include "Event.hpp"

#define SIMPLEDIM 5
#define COMPLEXDIM 3
#define NBPOLARITY 2

namespace Util {
    Eigen::Tensor<double, COMPLEXDIM> uniformMatrixComplex(long x, long y, long z, double normalizationFactor = 1);

    Eigen::Tensor<double, SIMPLEDIM> uniformMatrixSimple(long p, long c, long s, long x, long y, double normalizationFactor = 1);

    void normalizeSimpleTensor(Eigen::Tensor<double, SIMPLEDIM> &weights, double normalizationFactor);

    void normalizeComplexTensor(Eigen::Tensor<double, COMPLEXDIM> &weights, double normalizationFactor);

    void loadNumpyFileToSimpleTensor(Eigen::Tensor<double, SIMPLEDIM> &tensor, std::string &filePath);

    void loadNumpyFileToSimpleTensor(Eigen::Tensor<double, SIMPLEDIM> &tensor, cnpy::npz_t &arrayNPZ, std::string &arrayName);

    void loadNumpyFileToComplexTensor(Eigen::Tensor<double, COMPLEXDIM> &tensor, std::string &filePath);

    void loadNumpyFileToComplexTensor(Eigen::Tensor<double, COMPLEXDIM> &tensor, cnpy::npz_t &arrayNPZ, std::string &arrayName);

    void loadNumpyFileToWeights(std::unordered_map<size_t, double> &map, std::string &filePath);

    void loadNumpyFileToWeights(std::unordered_map<size_t, double> &map, cnpy::npz_t &arrayNPZ, std::string &arrayName);

    void saveSimpleTensorToNumpyFile(const Eigen::Tensor<double, SIMPLEDIM> &tensor, std::string &filePath);

    void saveSimpleTensorToNumpyFile(const Eigen::Tensor<double, SIMPLEDIM> &tensor, std::string &filePath, std::string &arrayName);

    void saveComplexTensorToNumpyFile(const Eigen::Tensor<double, COMPLEXDIM> &tensor, std::string &filePath);

    void saveComplexTensorToNumpyFile(const Eigen::Tensor<double, COMPLEXDIM> &tensor, std::string &filePath, std::string &arrayName);

    void saveWeightsToNumpyFile(const std::unordered_map<size_t, double> &map, std::string &filePath);

    void saveWeightsToNumpyFile(const std::unordered_map<size_t, double> &map, std::string &filePath, std::string &arrayName);

    int winnerTakeAll(std::vector<size_t> vec);

    double secondOrderNumericalDifferentiationMean(const std::vector<double> &vec, long n);

    bool fileExist(std::string &filePath);

    bool endsWith(std::string const & value, std::string const & ending);

    double ornsteinUhlenbeckProcess(double &pos, double dt, double theta, double mu, double sigma);

    void saveEventFile(std::vector<Event> &events, std::string &filePath);
}

class Luts {
public:
    Luts(double tauM, double tauRP, double tauSRA);

    std::vector<double> const lutM;
    std::vector<double> const lutRP;
    std::vector<double> const lutSRA;
private:
    static std::vector<double> expLUT(double tau);
};

class Position {
    uint64_t m_posx{};
    uint64_t m_posy{};
    uint64_t m_posz{};
public:
    inline Position() : m_posx(0), m_posy(0), m_posz(0) {};

    inline Position(uint64_t x, uint64_t y, uint64_t z) : m_posx(x), m_posy(y), m_posz(z) {}

    inline Position(uint64_t x, uint64_t y) : m_posx(x), m_posy(y), m_posz(0) {}

    [[nodiscard]] inline uint64_t x() const { return m_posx; }

    [[nodiscard]] inline uint64_t y() const { return m_posy; }

    [[nodiscard]] inline uint64_t z() const { return m_posz; }
};

#endif //NEUVISYS_DV_UTIL_HPP
