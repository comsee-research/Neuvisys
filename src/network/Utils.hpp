#ifndef NEUVISYS_DV_UTILS_HPP
#define NEUVISYS_DV_UTILS_HPP

#include "../dependencies/unsupported/Eigen/CXX11/Tensor"
#include <random>
#include <chrono>
#include "cnpy.h"

#define SIMPLEDIM 5
#define COMPLEXDIM 3
#define NBPOLARITY 2

namespace Util {
    Eigen::Tensor<double, COMPLEXDIM> uniformMatrixComplex(long x, long y, long z);
    Eigen::Tensor<double, SIMPLEDIM> uniformMatrixSimple(long p, long c, long s, long x, long y);
    void loadNumpyFileToSimpleTensor(Eigen::Tensor<double, SIMPLEDIM> &tensor, std::string &filePath);
    void saveSimpleTensorToNumpyFile(Eigen::Tensor<double, SIMPLEDIM> tensor, std::string &saveFile);
    void loadNumpyFileToComplexTensor(Eigen::Tensor<double, COMPLEXDIM> &tensor, std::string &filePath);
    void saveComplexTensorToNumpyFile(Eigen::Tensor<double, COMPLEXDIM> tensor, std::string &saveFile);
    int winnerTakeAll(std::vector<size_t> vec);
    void loadNumpyToWeights(std::unordered_map<size_t, double> &map, std::string &filePath);
    void saveWeightsToNumpy(std::unordered_map<size_t, double> &map, std::string &saveFile);
    double secondOrderNumericalDifferentiationMean(const std::vector<double> &vec, long n);
    bool fileExist(std::string &path);
    double ornsteinUhlenbeckProcess(double &pos, double dt, double theta, double mu, double sigma);
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

#endif //NEUVISYS_DV_UTILS_HPP
